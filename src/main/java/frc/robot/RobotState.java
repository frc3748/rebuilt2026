package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.game.AllianceFlip;
import frc.robot.game.BallTargetFactory;
import frc.robot.game.DashboardManager;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameState;
import frc.robot.game.PassTargetFactory;
import frc.robot.game.ShooterSetpoint;
import frc.robot.game.ShotCalculator;
import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.BatteryTracker;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.RobotTime;
import frc.robot.util.SimulatedRobotState;
import frc.robot.util.state.StateMachine;

public class RobotState extends StateMachine<RobotState.State> {
    private static final double kShiftWarningSeconds = 3.0;

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
    private final ShooterConstants shooterConstants;
    private final ShotCalculator shotCalculator;
    private final GameState gameState = new GameState();
    private final BatteryTracker battery = new BatteryTracker();
    private final Supplier<ShooterSetpoint> hubSupplier = ShooterSetpoint.hubSetpointSupplier(this);
    private final Supplier<ShooterSetpoint> passSupplier = ShooterSetpoint.passSetpointSupplier(this);
    private long setpointTimestamp = -1;
    private ShooterSetpoint hubSetpoint;
    private ShooterSetpoint passSetpoint;

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
        shooterConstants = definition.shooter();
        shotCalculator = new ShotCalculator(shooterConstants);
        drive = new Drive(definition.drive(), this);
        vision = new Vision(this, definition.cameras());
        superstructure = definition.createSuperstructure(this);
        dashboard = new DashboardManager(this, gameState, definition.name(), definition.autos(this));

        controls.bind(this);
        new Trigger(gameState::isHubActive).onChange(controls.rumble(0.5));
        new Trigger(() -> DriverStation.isTeleopEnabled() && gameState.isHubActiveNext() != gameState.isHubActive()
                && gameState.getSecondsUntilShift() > 0 && gameState.getSecondsUntilShift() <= kShiftWarningSeconds)
                .onTrue(controls.pulse(kShiftWarningSeconds));
        new Trigger(() -> DriverStation.isTeleopEnabled()
                && superstructure.getShooter().map(Shooter::isReadyToShoot).orElse(false))
                .debounce(0.1)
                .onTrue(controls.driverBuzz(0.2));

        addOmniTransitions(State.SOFT_STOP, State.TRAVERSING, State.AUTO);
        registerStateCommand(State.SOFT_STOP, drive.transitionCommand(Drive.State.IDLE));
        registerStateCommand(State.TRAVERSING, drive.transitionCommand(Drive.State.TRAVERSING));
        registerStateCommand(State.AUTO, drive.transitionCommand(Drive.State.TRAVERSING));

        addChildSubsystem(vision);
        addChildSubsystem(drive);
        superstructure.subsystems().forEach(this::addChildSubsystem);
        enable();
    }

    @Override
    protected void update() {
        if (Math.abs(controls.driver().getRightX()) > 0.1) {
            drive.requestTransition(Drive.State.TRAVERSING);
        }
        gameState.update();
        battery.update();
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
            SimulatedArena.getInstance().simulationPeriodic();
            superstructure.simulationPeriodic();
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
        drive.addVisionMeasurement(measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
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
        refreshSetpoints();
        if (hubSetpoint == null) {
            hubSetpoint = hubSupplier.get();
        }
        return hubSetpoint;
    }

    public ShooterSetpoint getCurrentPassSetpoint() {
        refreshSetpoints();
        if (passSetpoint == null) {
            passSetpoint = passSupplier.get();
        }
        return passSetpoint;
    }

    private void refreshSetpoints() {
        long now = Logger.getTimestamp();
        if (now != setpointTimestamp) {
            setpointTimestamp = now;
            hubSetpoint = null;
            passSetpoint = null;
        }
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

    public ShooterConstants getShooterConstants() {
        return shooterConstants;
    }

    public ShotCalculator getShotCalculator() {
        return shotCalculator;
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

    public BatteryTracker getBattery() {
        return battery;
    }

    public List<String> disconnectedDevices() {
        List<String> found = new ArrayList<>(drive.disconnectedDevices());
        if (Constants.kMode == Mode.REAL) {
            superstructure.subsystems().forEach(machine -> collectDisconnected(machine, found));
        }
        found.addAll(vision.disconnectedCameras());
        return found;
    }

    private static void collectDisconnected(StateMachine<?> machine, List<String> found) {
        machine.getMotors().stream()
                .filter(motor -> !motor.isConnected())
                .forEach(motor -> found.add(motor.getName().replace("Motors/", "")));
        machine.getChildSubsystems().forEach(child -> collectDisconnected(child, found));
    }

    public SimulatedRobotState getSimRobot() {
        return simulatedRobotState;
    }
}
