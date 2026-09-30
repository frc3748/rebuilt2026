package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.game.FuelSimulation;
import frc.robot.game.ShooterSetpoint;
import frc.robot.game.ShotVisualizer;
import frc.robot.util.state.StateMachine;

public class Shooter extends StateMachine<Shooter.State> {
    private final RobotState state;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Hopper hopper;
    private final Kicker kicker;
    private final FuelSimulation fuel;
    private final ShotVisualizer visualizer;
    private final Timer simShotTimer = new Timer();

    public Shooter(RobotState state, Flywheel flywheel, Hood hood, Hopper hopper, Kicker kicker, FuelSimulation fuel) {
        super("Shooter", State.UNDETERMINED, State.class);
        this.state = state;
        this.flywheel = flywheel;
        this.hood = hood;
        this.hopper = hopper;
        this.kicker = kicker;
        this.fuel = fuel;
        visualizer = new ShotVisualizer(state);

        addChildSubsystem(hood);
        addChildSubsystem(flywheel);

        allowAllTransitions();
        registerStateCommands();

        for (double distance : ShooterConstants.kShotDistances) {
            DogLog.tunable("TOF Tuning/" + distance, ShooterConstants.kTimeOfFlightMap.get(distance),
                    tof -> ShooterConstants.kTimeOfFlightMap.put(distance, tof));
        }

        simShotTimer.start();
        enable();
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE,
                () -> request(Flywheel.State.IDLE, Hood.State.IDLE, Hopper.State.IDLE, Kicker.State.IDLE));
        registerStateCommand(State.HUB_TRACKING,
                () -> request(Flywheel.State.TRACKING, Hood.State.HUB_TRACKING, Hopper.State.IDLE, Kicker.State.IDLE));
        registerStateCommand(State.PASS_TRACKING,
                () -> request(Flywheel.State.TRACKING, Hood.State.PASS_TRACKING, Hopper.State.IDLE, Kicker.State.IDLE));
        registerStateCommand(State.SHOOTING,
                () -> request(Flywheel.State.SHOOT, Hood.State.HUB_TRACKING, Hopper.State.SHOOT, Kicker.State.SHOOT));
        registerStateCommand(State.PASSING,
                () -> request(Flywheel.State.PASS, Hood.State.PASS_TRACKING, Hopper.State.SHOOT, Kicker.State.SHOOT));
        registerStateCommand(State.OUTTAKE,
                () -> request(Flywheel.State.IDLE, Hood.State.IDLE, Hopper.State.OUTAKE, Kicker.State.OUTAKE));
        registerStateCommand(State.TUNING,
                () -> request(Flywheel.State.TUNING, Hood.State.TUNING, Hopper.State.IDLE, Kicker.State.IDLE));
    }

    private void request(Flywheel.State flywheelState, Hood.State hoodState, Hopper.State hopperState,
            Kicker.State kickerState) {
        flywheel.requestTransition(flywheelState);
        hood.requestTransition(hoodState);
        hopper.requestTransition(hopperState);
        kicker.requestTransition(kickerState);
    }

    @Override
    protected void update() {
        if (Constants.kMode == Mode.SIM && fuel != null) {
            simulateShots();
        }
    }

    private void simulateShots() {
        ShooterSetpoint setpoint = isPassing() ? state.getCurrentPassSetpoint() : state.getCurrentHubSetpoint();
        LinearVelocity exitVelocity = MetersPerSecond.of(setpoint.getShooterRPS());
        Angle launchAngle = Degrees.of(90).minus(Radians.of(setpoint.getHoodRadians()));
        visualizer.update(exitVelocity, launchAngle);

        boolean firing = getState() == State.SHOOTING || getState() == State.PASSING;
        if (firing && simShotTimer.hasElapsed(ShooterConstants.kSimSecondsBetweenShots) && fuel.launch(exitVelocity, launchAngle)) {
            simShotTimer.reset();
        }
    }

    private boolean isPassing() {
        return getState() == State.PASSING || getState() == State.PASS_TRACKING;
    }

    public void holdShot(ShooterSetpoint setpoint, boolean spinFlywheel) {
        flywheel.setOverride(() -> flywheel.spin(spinFlywheel ? setpoint.getShooterRPS() : 0.0));
        hood.setOverride(() -> hood.aim(setpoint));
    }

    public void releaseShot() {
        flywheel.clearOverride();
        hood.clearOverride();
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        HUB_TRACKING,
        PASS_TRACKING,
        SHOOTING,
        PASSING,
        OUTTAKE,
        TUNING
    }
}
