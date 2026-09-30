package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.ShotVisualizer;
import frc.robot.util.state.StateMachine;

public class Shooter extends StateMachine<Shooter.State> {
    private final RobotState state;
    private final Hood hood;
    private final Flywheel flywheel;
    private final ShotVisualizer visualizer;
    private final Timer simShotTimer = new Timer();

    public Shooter(RobotState state) {
        super("Shooter", State.UNDETERMINED, State.class);
        this.state = state;
        hood = new Hood(state);
        flywheel = new Flywheel(state);
        visualizer = new ShotVisualizer(state);

        addChildSubsystem(hood);
        addChildSubsystem(flywheel);

        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING,
                State.SHOOTING, State.PASSING, State.OUTTAKE, State.TUNING);
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
        state.getHopper().requestTransition(hopperState);
        state.getKicker().requestTransition(kickerState);
    }

    @Override
    protected void update() {
        if (Constants.kMode == Mode.SIM) {
            simulateShots();
        }
    }

    private void simulateShots() {
        ShooterSetpoint setpoint = isPassing() ? state.getCurrentPassSetpoint() : state.getCurrentHubSetpoint();
        LinearVelocity exitVelocity = MetersPerSecond.of(setpoint.getShooterRPS());
        Angle launchAngle = Degrees.of(90).minus(Radians.of(setpoint.getHoodRadians()));
        visualizer.update(exitVelocity, launchAngle);

        boolean firing = getState() == State.SHOOTING || getState() == State.PASSING;
        if (firing && state.getSimFuelCount() > 0 && simShotTimer.hasElapsed(ShooterConstants.kSimSecondsBetweenShots)) {
            state.setSimFuelCount(state.getSimFuelCount() - 1);
            simShotTimer.reset();
            state.getFuelSim().launchFuel(
                    exitVelocity,
                    launchAngle,
                    Radians.zero(),
                    Meters.of(VisionConstants.kShooterToRobotCenter.getZ()));
        }
    }

    private boolean isPassing() {
        return getState() == State.PASSING || getState() == State.PASS_TRACKING;
    }

    public boolean isReady() {
        return flywheel.isReady();
    }

    public void setOverride(ShooterSetpoint setpoint, boolean spinFlywheel) {
        flywheel.setOverride(() -> flywheel.spin(spinFlywheel ? setpoint.getShooterRPS() : 0.0));
        hood.setOverride(() -> hood.aim(setpoint));
    }

    public void clearOverride() {
        flywheel.setOverride(null);
        hood.setOverride(null);
    }

    public Hood getHood() {
        return hood;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
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
