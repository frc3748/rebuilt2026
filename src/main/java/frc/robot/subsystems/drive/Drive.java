package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPath;
import frc.robot.util.RobotTime;
import frc.robot.util.TunableNumber;
import frc.robot.game.ShotCalculator;
import frc.robot.game.TrenchZone;
import frc.robot.util.state.StateMachine;
import frc.robot.util.tuning.Source;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachine<Drive.State> {
  private static final Pose2d kSimStartPose = new Pose2d(2.5, 2.0, Rotation2d.kZero);
  private static final double kAimTargetFreshSeconds = 0.25;
  private Translation2d aimTarget;
  private double aimTargetTime = Double.NEGATIVE_INFINITY;

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules = new Module[4];
  private Pose2d pathTarget;
  private PIDConstants pathTranslation;
  private PIDConstants pathRotation;
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert;

  private final DriveConfig config;
  private final TunableNumber slowSpeed;
  private final SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveSimulation simulation;
  private final SlipCorrector slip;
  private final CollisionDetector collision = new CollisionDetector();
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint setpoint;
  private boolean setpointLive;
  private final SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
  private double lastOdometryTimestamp = Double.NaN;

  private RobotState robotState;

  public Drive(DriveConfig config, RobotState robotState) {
    super("Drive", State.UNDETERMINED, State.class);
    this.config = config;
    gyroDisconnectedAlert = new Alert(Module.kAlertGroup, gyroName() + " disconnected, heading comes from the wheels", AlertType.kError);
    config.wheelRadiusMeters = TunableNumber.field("Drive/Wheel Radius", config, "wheelRadiusMeters").restartToApply().get();
    slowSpeed = TunableNumber.field("Drive/Slow Speed", config, "slowSpeedMetersPerSec");
    this.robotState = robotState;
    kinematics = new SwerveDriveKinematics(config.moduleTranslations());
    slip = new SlipCorrector(config.moduleTranslations());
    setpointGenerator = new SwerveSetpointGenerator(config.pathPlannerConfig(), config.maxSteerVelocity());
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, odometryPositions, Pose2d.kZero);

    simulation = Constants.kMode == Mode.SIM ? DriveSimulation.create(config, kSimStartPose) : null;
    if (simulation != null) {
      poseEstimator.resetPosition(rawGyroRotation, odometryPositions, kSimStartPose);
    }
    gyroIO = createGyro(config, simulation);
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(createModule(config, i, simulation), i, config.wheelRadiusMeters, config.maxSteerVelocity());
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

  }

  private static GyroIO createGyro(DriveConfig config, SwerveDriveSimulation simulation) {
    if (simulation != null) {
      return new GyroIOSim(simulation);
    }
    if (Constants.kMode != Mode.REAL) {
      return new GyroIO() {};
    }
    return switch (config.gyro) {
      case PIGEON2 -> new GyroIOPigeon2(config);
      case NAVX -> new GyroIONavX(config);
    };
  }

  private static ModuleIO createModule(DriveConfig config, int index, SwerveDriveSimulation simulation) {
    return switch (Constants.kMode) {
      case REAL -> new ModuleIOSpark(config, index);
      case SIM -> new ModuleIOSim(config, simulation.getModules()[index]);
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
          if (currentState == State.SLOW && config.aimInSlowMode) {
            return State.TRAVERSING_AT_ANGLE;
          }

          return currentState;
        }
    ));

    registerStateCommand(State.CROSSED, new InstantCommand(() -> stopWithX()));
  }

  private void configureAutobuilder() {
    pathTranslation = pathPid("Path/Translation", "pathTranslationPid", config.pathTranslationPid);
    pathRotation = pathPid("Path/Rotation", "pathRotationPid", config.pathRotationPid);
    AutoBuilder.configureCustom(this::followPath, this::getPose, this::setPose, Drive::isRedAlliance, true);
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          pathTarget = targetPose;
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public FollowPath followPath(PathPlannerPath path) {
    return new FollowPath(this, path, pathTranslation, pathRotation, config.pathPlannerConfig(), Drive::isRedAlliance);
  }

  public Optional<Translation2d> getRecentAimTarget() {
    return Timer.getFPGATimestamp() - aimTargetTime < kAimTargetFreshSeconds ? Optional.ofNullable(aimTarget) : Optional.empty();
  }

   public Rotation2d getAimRotationForHub() {
    if (TrenchZone.driveRotationOverrideRequired(robotState) && getState() == State.SLOW) {
      Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
      double degrees = currentPose.getRotation().getDegrees();
      return (Math.abs(degrees) <= 90) ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    Pose2d currentPose = getPose().plus(
      new Transform2d(robotState.getShooterConstants().shooterToRobotCenter.getTranslation().toTranslation2d(), Rotation2d.kZero)
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

    Logger.recordOutput("Drive/AimTarget", new Pose2d(targetTrans, goal));
    aimTarget = targetTrans;
    aimTargetTime = Timer.getFPGATimestamp();

    return goal;
  }

  @Override
  public void update() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);

    double timestamp = RobotTime.getTimestampSeconds();
    robotState.addOdometryMeasurement(timestamp, getPose());

    recordMotion(timestamp);
    if (simulation != null) {
      robotState.getSimRobot().addFieldToRobot(simulation.getSimulatedDriveTrainPose());
    }

    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      setpointLive = false;
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    }

    ChassisSpeeds wheelSpeeds = getChassisSpeeds();
    slip.update(wheelSpeeds, gyroInputs.connected, gyroInputs.accelXGs, gyroInputs.accelYGs,
        gyroInputs.pitchRadians, gyroInputs.rollRadians, 0.02);
    collision.update(wheelSpeeds, gyroInputs.connected, gyroInputs.accelXGs, gyroInputs.accelYGs,
        gyroInputs.pitchRadians, gyroInputs.rollRadians, timestamp, 0.02);

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

      Rotation2d previousGyro = rawGyroRotation;
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      double dt = Double.isNaN(lastOdometryTimestamp) ? 0.02 : Math.max(1e-3, sampleTimestamps[i] - lastOdometryTimestamp);
      lastOdometryTimestamp = sampleTimestamps[i];
      SwerveModulePosition[] corrected = gyroInputs.connected
          ? slip.correct(moduleDeltas, rawGyroRotation.minus(previousGyro).getRadians(), dt)
          : moduleDeltas;
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        odometryPositions[moduleIndex] = new SwerveModulePosition(
            odometryPositions[moduleIndex].distanceMeters + corrected[moduleIndex].distanceMeters, corrected[moduleIndex].angle);
      }
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, odometryPositions);
    }

    Logger.recordOutput("Drive/Slip/Modules", slip.slippingModules());
    Logger.recordOutput("Drive/Slip/Robot", slip.isRobotSlipping());
    Logger.recordOutput("Drive/Slip/Scale", slip.scale());
    Logger.recordOutput("Drive/Slip/EstimatedSpeed", slip.estimatedSpeed());
    Logger.recordOutput("Drive/Slip/Events", slip.events());
    Logger.recordOutput("Drive/Collision/Jolt", collision.jolt());
    Logger.recordOutput("Drive/Collision/Hit", collision.isHit());
    Logger.recordOutput("Drive/Collision/Tilted", collision.isTilted());
    Logger.recordOutput("Drive/Collision/TrustingVision", collision.isUpset());
    Logger.recordOutput("Drive/Collision/Events", collision.events());

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.kMode != Mode.SIM);
  }

  private void recordMotion(double timestamp) {
    ChassisSpeeds measured = getChassisSpeeds();
    ChassisSpeeds measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(measured, getPose().getRotation());
    ChassisSpeeds desiredField = ChassisSpeeds.fromRobotRelativeSpeeds(desiredSpeeds, getPose().getRotation());
    ChassisSpeeds fusedField = new ChassisSpeeds(
        measuredField.vxMetersPerSecond, measuredField.vyMetersPerSecond, gyroInputs.yawRateRadPerSec);

    robotState.addDriveMotionMeasurements(timestamp,
        gyroInputs.rollRateRadPerSec, gyroInputs.pitchRateRadPerSec, gyroInputs.yawRateRadPerSec,
        gyroInputs.pitchRadians, gyroInputs.rollRadians, gyroInputs.accelXGs, gyroInputs.accelYGs,
        desiredField, measured, measuredField, fusedField);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    if (!config.useSetpointGenerator) {
      runVelocity(discreteSpeeds, new double[4]);
      return;
    }
    if (!setpointLive) {
      setpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
    }
    setpoint = setpointGenerator.generateSetpoint(setpoint, discreteSpeeds, 0.02);
    Logger.recordOutput("SwerveChassisSpeeds/Requested", discreteSpeeds);
    runModules(setpoint.robotRelativeSpeeds(), setpoint.moduleStates(), setpoint.feedforwards().accelerationsMPSSq());
    setpointLive = true;
  }

  public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    runVelocity(ChassisSpeeds.discretize(speeds, 0.02), feedforwards.accelerationsMPSSq());
  }

  private void runVelocity(ChassisSpeeds discreteSpeeds, double[] accelerations) {
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, config.maxSpeedMetersPerSec);
    runModules(discreteSpeeds, setpointStates, accelerations);
    setpoint = new SwerveSetpoint(discreteSpeeds, setpointStates, DriveFeedforwards.zeros(4));
    setpointLive = true;
  }

  private void runModules(ChassisSpeeds speeds, SwerveModuleState[] states, double[] accelerations) {
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);
    Logger.recordOutput("SwerveStates/Setpoints", states);
    Logger.recordOutput("SwerveStates/AccelerationFeedforwards", accelerations);
    SwerveModuleState[] commanded = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      commanded[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
      modules[i].runSetpoint(commanded[i], accelerations[i]);
    }
    desiredSpeeds = speeds;
  }

  public void runCharacterization(double output) {
    setpointLive = false;
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void runModuleAngles(Rotation2d angle) {
    setpointLive = false;
    for (Module module : modules) {
      module.runAngle(angle);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds(), new double[4]);
  }

  public void liftDriveCurrentLimit(double amps) {
    for (Module module : modules) {
      module.liftDriveCurrentLimit(amps);
    }
  }

  public void restoreDriveCurrentLimit() {
    for (Module module : modules) {
      module.restoreDriveCurrentLimit();
    }
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
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getWheelSpeedMetersPerSec() {
    double total = 0.0;
    for (Module module : modules) {
      total += module.getVelocityMetersPerSec();
    }
    return total / 4.0;
  }

  public double getDriveCurrentAmps() {
    double total = 0.0;
    for (Module module : modules) {
      total += module.getDriveCurrentAmps();
    }
    return total / 4.0;
  }

  public double getDriveAppliedVolts() {
    double total = 0.0;
    for (Module module : modules) {
      total += module.getDriveAppliedVolts();
    }
    return total / 4.0;
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

   @AutoLogOutput(key = "Odometry/Robot3D")
  public Pose3d get3dPose() {
    return new Pose3d(getPose());
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Rotation2d getGyroRotation() {
    return rawGyroRotation;
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, odometryPositions, pose);
    robotState.resetBuffersToPose(pose);
    if (simulation != null) {
      simulation.setSimulationWorldPose(pose);
    }
  }

  public SlipCorrector getSlip() {
    return slip;
  }

  public CollisionDetector getCollision() {
    return collision;
  }

  public Optional<SwerveDriveSimulation> getSimulation() {
    return Optional.ofNullable(simulation);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs.times(collision.visionStdDevScale()));
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds);
  }

  private PIDConstants pathPid(String key, String field, PIDConstants defaults) {
    double kP = new TunableNumber(key + " kP", defaults.kP, Source.field(config, field, "PIDConstants", 0)).restartToApply().get();
    double kI = new TunableNumber(key + " kI", defaults.kI, Source.field(config, field, "PIDConstants", 1)).restartToApply().get();
    double kD = new TunableNumber(key + " kD", defaults.kD, Source.field(config, field, "PIDConstants", 2)).restartToApply().get();
    return new PIDConstants(kP, kI, kD);
  }

  public RobotConfig getPathPlannerConfig() {
    return config.pathPlannerConfig();
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return getState() == State.SLOW ? slowSpeed.get() : config.maxSpeedMetersPerSec;
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

  public void determineSelf() {
    setState(State.TRAVERSING);
  }

  public Optional<Pose2d> getPathTarget() {
    return Optional.ofNullable(pathTarget);
  }

  public void zeroHeading() {
    setPose(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
  }

  public List<String> disconnectedDevices() {
    List<String> found = new ArrayList<>();
    for (Module module : modules) {
      found.addAll(module.disconnected());
    }
    if (!gyroInputs.connected && Constants.kMode == Mode.REAL) {
      found.add(gyroName());
    }
    return found;
  }

  public String gyroName() {
    return config.gyro == DriveConfig.GyroType.NAVX ? "NavX" : "Pigeon";
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