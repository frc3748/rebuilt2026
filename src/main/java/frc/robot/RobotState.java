package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.game.AllianceFlip;
import frc.robot.game.BallTargetFactory;
import frc.robot.game.DashboardManager;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameState;
import frc.robot.game.PassTargetFactory;
import frc.robot.game.ShooterSetpoint;
import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.RobotTime;
import frc.robot.util.SimulatedRobotState;
import frc.robot.util.state.StateMachine;

public class RobotState extends StateMachine<RobotState.State> {
    public enum State {
        UNDETERMINED,
        SOFT_STOP,
        TRAVERSING,
        AUTO
    }

    public static final double LOOKBACK_TIME = 1.0;

    private final RobotDefinition definition;
    private final SimulatedRobotState simulatedRobotState = Robot.isSimulation() ? new SimulatedRobotState() : null;
    private final Controls controls;
    private final GameState gameState = new GameState();
    private final Supplier<ShooterSetpoint> hubSupplier = ShooterSetpoint.hubSetpointSupplier(this);
    private final Supplier<ShooterSetpoint> passSupplier = ShooterSetpoint.passSetpointSupplier(this);

    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;
    private final DashboardManager dashboard;

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());

    public RobotState(RobotDefinition definition) {
        super("RobotState", State.UNDETERMINED, State.class);
        this.definition = definition;
        clearBuffers();

        controls = definition.createControls();
        drive = new Drive(definition.drive(), this);
        vision = new Vision(this, definition.cameras());
        superstructure = definition.createSuperstructure(this);
        dashboard = new DashboardManager(this, gameState, definition.name(), definition.autos(this));

        controls.bind(this);
        new Trigger(gameState::isHubActive).onChange(controls.rumble(0.5));
        CameraServer.startAutomaticCapture();

        addOmniTransitions(State.SOFT_STOP, State.TRAVERSING, State.AUTO);
        registerStateCommand(State.SOFT_STOP, drive.transitionCommand(Drive.State.IDLE));
        registerStateCommand(State.TRAVERSING, drive.transitionCommand(Drive.State.TRAVERSING));
        registerStateCommand(State.AUTO, drive.transitionCommand(Drive.State.TRAVERSING));

        addChildSubsystem(vision);
        addChildSubsystem(drive);
        superstructure.subsystems().forEach(this::addChildSubsystem);
        enable();

        Logger.recordOutput("Bumper/Pose", new Pose3d());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    @Override
    protected void update() {
        if (Math.abs(controls.driver().getRightX()) > 0.1) {
            drive.requestTransition(Drive.State.TRAVERSING);
        }
        gameState.update();
        dashboard.update();
    }

    @Override
    protected void onTeleopStart() {
        setState(State.TRAVERSING);
        dashboard.clearPreview();
    }

    @Override
    protected void onAutonomousStart() {
        registerStateCommand(State.AUTO, dashboard.getSelectedAuto().build());
        dashboard.clearPreview();
        setState(State.AUTO);
    }

    @Override
    protected void determineSelf() {
        setState(State.TRAVERSING);
    }

    public void updateSimulation() {
        if (Constants.kMode == Mode.SIM) {
            superstructure.simulationPeriodic();
        }
    }

    public void updateLogger() {
        logLatest("RobotState/YawAngularVelocity", driveYawAngularVelocity);
        logLatest("RobotState/RollAngularVelocity", driveRollAngularVelocity);
        logLatest("RobotState/PitchAngularVelocity", drivePitchAngularVelocity);
        logLatest("RobotState/AccelX", accelX);
        logLatest("RobotState/AccelY", accelY);
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());
    }

    private static void logLatest(String key, ConcurrentTimeInterpolatableBuffer<Double> buffer) {
        var latest = buffer.getInternalBuffer().lastEntry();
        if (latest != null) {
            Logger.recordOutput(key, latest.getValue());
        }
    }

    public void clearBuffers() {
        fieldToRobot.clear();
        driveYawAngularVelocity.clear();
        fieldToRobot.addSample(0.0, Pose2d.kZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    public void resetBuffersToPose(Pose2d pose) {
        fieldToRobot.clear();
        fieldToRobot.addSample(Timer.getFPGATimestamp(), pose);
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
        driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.accelX.addSample(timestamp, accelX);
        this.accelY.addSample(timestamp, accelY);
        desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public void addVisionMeasurement(VisionMeasurement measurement) {
        if (Constants.kMode == Mode.REAL) {
            drive.addVisionMeasurement(measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
        }
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        fieldToRobot.addSample(RobotTime.getTimestampSeconds(), drive.getPose());
        return fieldToRobot.getLatest();
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
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

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if (Constants.kMode != Mode.REAL) {
            return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
        }
        var values = driveYawAngularVelocity.getInternalBuffer().subMap(minTime, maxTime).values();
        return values.stream().max((a, b) -> Double.compare(Math.abs(a), Math.abs(b)));
    }

    public ShooterSetpoint getCurrentHubSetpoint() {
        return hubSupplier.get();
    }

    public ShooterSetpoint getCurrentPassSetpoint() {
        return passSupplier.get();
    }

    public boolean shouldShootHub() {
        double x = getLatestFieldToRobot().getValue().getX();
        return AllianceFlip.isRed() ? x >= FieldConstants.HUB_RED.getX() : x <= FieldConstants.HUB_BLUE.getX();
    }

    public Pose2d getDriveAnglePos() {
        Translation3d target = shouldShootHub() ? BallTargetFactory.generate(this) : PassTargetFactory.generate(this);
        return new Pose2d(target.toTranslation2d(), Pose2d.kZero.getRotation());
    }

    public RobotDefinition getDefinition() {
        return definition;
    }

    public Drive getDrive() {
        return drive;
    }

    public Vision getVision() {
        return vision;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public Controls getControls() {
        return controls;
    }

    public GameState getGameState() {
        return gameState;
    }

    public SimulatedRobotState getSimRobot() {
        return simulatedRobotState;
    }
}
