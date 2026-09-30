package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.util.Elastic;
import frc.robot.util.RobotTime;
import frc.robot.game.ShotCalculator;
import frc.robot.game.TrenchZone;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.state.StateMachine;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachine<Drive.State> implements DriveIO {
  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();

  public final Field2d fieldPose = new Field2d();

  private final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private final DriveConfig config;
  private final SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator;

  private RobotState robotState;

  public Drive(DriveConfig config, RobotState robotState) {
    super("Drive", State.UNDETERMINED, State.class);
    this.config = config;
    this.robotState = robotState;
    kinematics = new SwerveDriveKinematics(config.moduleTranslations());
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

    gyroIO = createGyro(config);
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(createModule(config, i), i, config.wheelRadiusMeters);
    }

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    if (RobotBase.isReal()) {
      SparkOdometryThread.getInstance().start(config.odometryFrequency);
    }

    configureAutobuilder();

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    registerStateTransitions();
    registerStateCommands();
    enable();

    SmartDashboard.putData("Swerve Zero", Commands.runOnce(
        () -> this.setPose(
            new Pose2d(this.getPose().getTranslation(), Rotation2d.kZero)),
        this)
        .ignoringDisable(true));

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
      }
    });
  }

  private static GyroIO createGyro(DriveConfig config) {
    if (Constants.kMode != Mode.REAL) {
      return new GyroIO() {};
    }
    return switch (config.gyro) {
      case PIGEON2 -> new GyroIOPigeon2(config);
      case NAVX -> new GyroIONavX(config);
    };
  }

  private static ModuleIO createModule(DriveConfig config, int index) {
    return switch (Constants.kMode) {
      case REAL -> new ModuleIOSpark(config, index);
      case SIM -> new ModuleIOSim(config);
      case REPLAY -> new ModuleIO() {};
    };
  }

  private void registerStateTransitions() {
    addOmniTransitions(State.IDLE, State.CROSSED, State.ALIGNING, State.PATHFINDING, State.SLOW, State.TRAVERSING,
        State.TRAVERSING_AT_ANGLE, State.UNDETERMINED);
  }

  private void registerStateCommands() {
    registerStateCommand(State.IDLE, new InstantCommand(() -> stop()));

    setDefaultCommand(DriveCommands.smartDrive(
        this,
        () -> -robotState.getControls().driver().getLeftY(),
        () -> -robotState.getControls().driver().getLeftX(),
        () -> -robotState.getControls().driver().getRightX(),
        this::getAimRotationForHub,
        () -> {
          State currentState = getState();
          if (currentState == State.SLOW) {
            return State.TRAVERSING_AT_ANGLE;
          }

          return currentState;
        }
    ));

    registerStateCommand(State.CROSSED, new InstantCommand(() -> stopWithX()));
  }

  private void configureAutobuilder() {
    Elastic.sendNotification(new Notification().withTitle("Auto Builder").withDescription("Auto builder reset"));
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            config.pathTranslationPid, config.pathRotationPid),
        config.pathPlannerConfig(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

   public Rotation2d getAimRotationForHub() {
    if (TrenchZone.driveRotationOverrideRequired(robotState) && getState() == State.SLOW) {
      Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
      double degrees = currentPose.getRotation().getDegrees();
      return (Math.abs(degrees) <= 90) ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    Pose2d currentPose = getPose().plus(
      new Transform2d(ShooterConstants.kShooterToRobotCenter.getTranslation().toTranslation2d(), Rotation2d.kZero)
    );
    Translation2d targetTrans = robotState.getDriveAnglePos().getTranslation();
    double distance = currentPose.getTranslation().getDistance(targetTrans);
    double shotExitVelocity = robotState.getCurrentHubSetpoint().getShooterRPS();

    double timeOfFlight = ((distance) / shotExitVelocity) * 3;

    if (!DriverStation.isAutonomous()) {
      targetTrans = ShotCalculator.predictTargetPos(new Translation3d(targetTrans), getChassisSpeeds(), Seconds.of(timeOfFlight)).toTranslation2d();
    }

    Translation2d drivingVector = targetTrans.minus(currentPose.getTranslation());
    Rotation2d goal = drivingVector.getAngle();

    driveInputs.driveAtAngleGoal = new Pose2d(targetTrans, goal);
    driveInputs.driveAtAngleDesired = new Pose2d(currentPose.getX(), currentPose.getY(), goal);

    return goal;
  }

  @Override
  public void update() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);

    {
      driveInputs.modStates = getModuleStates();
      driveInputs.currentPose = getPose();
      driveInputs.currentPose3d = new Pose3d(driveInputs.currentPose);

      fieldPose.setRobotPose(driveInputs.currentPose);
    }

    double timestamp = RobotTime.getTimestampSeconds();
    robotState.addOdometryMeasurement(timestamp, getPose());

    if (Constants.kMode != Mode.SIM) {
      recordMotion(timestamp);
    } else {
      robotState.getSimRobot().addFieldToRobot(getPose());
    }

    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.processInputs("Drive/DriveBase", driveInputs);
    SmartDashboard.putData(fieldPose);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected);
  }

  private void recordMotion(double timestamp) {
    if (driveInputs.optimizedModStates.length != 4) {
      return;
    }
    ChassisSpeeds measured = kinematics.toChassisSpeeds(driveInputs.optimizedModStates);
    ChassisSpeeds measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(measured, getPose().getRotation());
    ChassisSpeeds desiredField = ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(driveInputs.modStates), getPose().getRotation());
    ChassisSpeeds fusedField = new ChassisSpeeds(
        measuredField.vxMetersPerSecond, measuredField.vyMetersPerSecond, gyroInputs.yawRateRadPerSec);

    robotState.addDriveMotionMeasurements(timestamp,
        gyroInputs.rollRateRadPerSec, gyroInputs.pitchRateRadPerSec, gyroInputs.yawRateRadPerSec,
        gyroInputs.pitchRadians, gyroInputs.rollRadians, gyroInputs.accelXGs, gyroInputs.accelYGs,
        desiredField, measured, measuredField, fusedField);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

    driveInputs.chassieSpeeds = discreteSpeeds;
    driveInputs.optimizedModStates = setpointStates;
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    Translation2d[] translations = config.moduleTranslations();
    for (int i = 0; i < 4; i++) {
      headings[i] = translations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    robotState.resetBuffersToPose(pose);
    Elastic.sendNotification(
        new Notification().withTitle("Pose Reset").withDescription("Pose has been set to a new custom one"));
  }

  public void setTargetPose(Pose2d pose) {
    driveInputs.goalPose = pose;
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return getState() == State.SLOW ? config.slowSpeedMetersPerSec : config.maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / config.driveBaseRadius();
  }

  public DriveConfig getConfig() {
    return config;
  }

  public GyroIOInputsAutoLogged getGyroIOInputs() {
    return gyroInputs;
  }

  public DriveIOInputsAutoLogged getDriveIOInputs() {
    return driveInputs;
  }

  public void determineSelf() {
    setState(State.TRAVERSING);
  }

  @Override
  public void onTeleopStart() {
    setFieldPoses();
  }

  public void setFieldPoses(Pose2d... poses) {
    fieldPose.getObject("mainTrajectory").setPoses(poses);
  }

  public void setFieldPoses(String object, List<Pose2d> poses) {
    fieldPose.getObject(object).setPoses(poses);
  }

  public enum State {
    UNDETERMINED,

    IDLE,
    CROSSED,
    TRAVERSING,
    TRAVERSING_AT_ANGLE,

    PATHFINDING,
    ALIGNING,

    SLOW,
  }
}