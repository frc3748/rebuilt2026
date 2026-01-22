package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.Elastic;
import frc.robot.util.MathHelpers;
import frc.robot.util.RobotTime;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.state.StateMachine;

public class RobotState extends StateMachine<RobotState.State> {

    public final static double LOOKBACK_TIME = 1.0;
    public final static AtomicBoolean hubActivated = new AtomicBoolean();

    private Drive drive;
    // private VisionSubsystem vision;

    private CommandXboxController controller = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    // kinematic frame
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Rotation2d> robotToTurret = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private static final Transform2d TURRET_TO_CAMERA = new Transform2d(VisionConstants.kTurretToCameraX,
            VisionConstants.kTurretToCameraY,
            MathHelpers.kRotation2dZero);
    private static final Transform2d ROBOT_TO_CAMERA_B = new Transform2d(VisionConstants.kTurretToCameraBX,
            VisionConstants.kTurretToCameraBY,
            MathHelpers.kRotation2dZero);
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());

    private double lastUsedMegatagTimestamp = 0;
    private double lastTriggeredIntakeSensorTimestamp = 0;
    private ConcurrentTimeInterpolatableBuffer<Double> turretAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> turretPositionRadians = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelX = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelY = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private double autoStartTime;

    public RobotState() {
        super("RobotState", State.UNDETERMINED, State.class);
        // drive intialization
        {
            drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    this);

            visionEstimateConsumer = new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate estimate) {
                    drive.addVisionMeasurement(estimate.getVisionRobotPoseMeters(), estimate.getTimestampSeconds());
                }
            };
        }

        // vision initialization TODO
        {
            // vision = new VisionSubsystem(new VisionIOHardwareLimelight(this), this);
        }

        // auto setup
        {
            autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
            setupDriveDiagnosisAutos();
        }

        setupControllerBindings();
        setupNotis();

        registerStateTransitions();
        registerStateCommands();

        addChildSubsystem(drive);
        // addChildSubsystem(vision);
        enable();
    }

    private void setupDriveDiagnosisAutos() {
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void setupNotis() {
        @SuppressWarnings("unused")
        Trigger notifyLowOnTime = new Trigger(() -> {
            // when in teleop tether mode the getMatchTime counts up. only do this when
            // counting down
            boolean isTeleopOnField = DriverStation.isTeleopEnabled() && DriverStation.isFMSAttached();
            double matchTime = DriverStation.getMatchTime();
            double ts = Timer.getFPGATimestamp();

            // teleop
            return isTeleopOnField &&
            // 25-35s
                    (matchTime >= 15.0 && matchTime <= 35.0) &&
            // for a small amount of time each second
                    ((ts - Math.floor(ts)) > 0.700);
        });
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.SOFT_STOP, State.TRAVERSING, State.AUTO, State.SHOOTING, State.SHOOTING_INTAKING,
                State.CLIMBING, State.INTAKING, State.PASSING, State.PASSING_INTAKING);
    }

    private void registerStateCommands() {
        registerStateCommand(State.SOFT_STOP, new ParallelCommandGroup(
                drive.transitionCommand(Drive.State.IDLE)));

        registerStateCommand(State.TRAVERSING, new ParallelCommandGroup(
                drive.transitionCommand(Drive.State.TRAVERSING)));

        // // change this to an auto state in the future?
        registerStateCommand(State.AUTO, new ParallelCommandGroup(
                drive.transitionCommand(Drive.State.TRAVERSING)));
    }

    private void setupControllerBindings() {

    }

    public Command getAutonomousCommand() {
        return Commands.print("None");
    }

    public Drive getDrive() {
        return drive;
    }

    public VisionSubsystem getVision() {
        return null;
        // return vision;
    }

    public CommandXboxController getController() {
        return controller;
    }

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void addDriveMotionMeasurements(double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot
                .exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    public Pose2d getLatestFieldToRobotCenter() {
        return fieldToRobot.getLatest().getValue().transformBy(VisionConstants.kTurretToRobotCenter);
    }

    // This has rotation and radians to allow for wrapping tracking.
    public void addTurretUpdates(double timestamp,
            Rotation2d turretRotation,
            double turretRadians,
            double angularYawRadsPerS) {
        // turret frame 180 degrees off from robot frame
        robotToTurret.addSample(timestamp, turretRotation.rotateBy(MathHelpers.kRotation2dPi));
        this.turretAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.turretPositionRadians.addSample(timestamp, turretRadians);
    }

    public double getLatestTurretPositionRadians() {
        return this.turretPositionRadians.getInternalBuffer().lastEntry().getValue();
    }

    public double getLatestTurretAngularVelocity() {
        return this.turretAngularVelocity.getInternalBuffer().lastEntry().getValue();
    }

    public Transform2d getTurretToCamera(boolean isTurretCamera) {
        return isTurretCamera ? TURRET_TO_CAMERA : ROBOT_TO_CAMERA_B;
    }

    public Optional<Rotation2d> getRobotToTurret(double timestamp) {
        return robotToTurret.getSample(timestamp);
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public Map.Entry<Double, Rotation2d> getLatestRobotToTurret() {
        return robotToTurret.getLatest();
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    public Optional<Double> getTurretAngularVelocity(double timestamp) {
        return turretAngularVelocity.getSample(timestamp);
    }

    private Optional<Double> getMaxAbsValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime,
            double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty())
            return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get()))
            return max;
        else
            return min;
    }

    public Optional<Double> getMaxAbsTurretYawAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(turretAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        // Gyro yaw rate not set in sim.
        if (Robot.isReal())
            return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = Timer.getFPGATimestamp();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public void updatePinholeEstimate(VisionFieldPoseEstimate pinholeEstimate) {
        visionEstimateConsumer.accept(pinholeEstimate);
    }

    public void updateLastTriggeredIntakeSensorTimestamp(boolean triggered) {
        if (triggered)
            lastTriggeredIntakeSensorTimestamp = RobotTime.getTimestampSeconds();
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public double lastTriggeredIntakeSensorTimestamp() {
        return lastTriggeredIntakeSensorTimestamp;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/YawAngularVelocity",
                    this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollAngularVelocity",
                    this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchAngularVelocity",
                    this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelX", this.accelX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelY", this.accelY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());
    }

    @Override
    protected void onTeleopStart() {
        requestTransition(State.TRAVERSING);
    }

    @Override
    protected void onAutonomousStart() {
        registerStateCommand(State.AUTO, autoChooser.get().andThen(new PrintCommand("Auto is Done!")));
        requestTransition(State.AUTO);
    }

    @Override
    protected void determineSelf() {
        setState(State.SOFT_STOP);
    }

    @Override
    public void update() {
        String gameState = "No Game State";
        double secondsUntilAllianceShift = 25;
        try {
            String message = DriverStation.getGameSpecificMessage();
            Optional<Alliance> teamAlliance = DriverStation.getAlliance();

            char autoWinner = (message.length() > 0) ? message.charAt(0) : ' ';
            double matchTime = DriverStation.getMatchTime();

            boolean inTransitionShift = (matchTime >= 130);
            boolean inEndGame = (matchTime <= 30);
            // ONLY refer to this if both booleans are false
            int currentStage = (4 - (int) ((matchTime - 30) / 25));

            if (DriverStation.isAutonomous()) {
                gameState = "Autonomous";
                secondsUntilAllianceShift = 0;
            } else if (inTransitionShift) {
                gameState = "Transition Shift";
                secondsUntilAllianceShift = matchTime - 130;
            } else if (inEndGame) {
                gameState = "End Game";
                secondsUntilAllianceShift = matchTime;
            } else {
                secondsUntilAllianceShift = (matchTime - 30) % 25;
                switch (currentStage) {
                    case 1:
                        gameState = "Alliance Shift 1";
                        break;
                    case 2:
                        gameState = "Alliance Shift 2";
                        break;
                    case 3:
                        gameState = "Alliance Shift 3";
                        break;
                    case 4:
                        gameState = "Alliance Shift 4";
                        break;
                    default:
                        gameState = "Teleop";
                        break;
                }
            }

            if (teamAlliance.isPresent() && message.length() > 0 && !inTransitionShift && !inEndGame
                    && DriverStation.isTeleop()) {
                char myColor = (teamAlliance.get() == Alliance.Red) ? 'R' : 'B';
                boolean isStageEven = (currentStage % 2 == 0);

                if (autoWinner == 'B') {
                    hubActivated.set(isStageEven ? (myColor == 'B') : (myColor == 'R'));
                } else if (autoWinner == 'R') {
                    hubActivated.set(isStageEven ? (myColor == 'R') : (myColor == 'B'));
                } else {
                    hubActivated.set(true);
                }
            } else {
                // UNKNOWN so have it activated to allow shooting
                hubActivated.set(true);
            }

            SmartDashboard.putBoolean("Game/HubActivated", hubActivated.get());
            SmartDashboard.putString("Game/GameState", gameState);
            SmartDashboard.putNumber("Game/ShiftCountdown", secondsUntilAllianceShift);
        } finally {
            Elastic.sendNotification(new Notification().withTitle("Error").withDescription("Failed to setup hub display logic"));
        }
    }

    public enum State {
        UNDETERMINED,
        SOFT_STOP,
        TRAVERSING,
        AUTO,

        CLIMBING,
        SHOOTING,
        SHOOTING_INTAKING,
        INTAKING,
        PASSING,
        PASSING_INTAKING
    }
}
