package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.game.ShooterSetpoint;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.util.TunableNumber;
import frc.robot.util.cockpit.Cockpit;

public class ShooterComp extends Shooter {
    protected final Flywheel flywheel;
    protected final Hood hood;
    protected final Hopper hopper;
    protected final Kicker kicker;

    private static final TunableNumber kAimToleranceRadians = new TunableNumber("Shooter/Aim Tolerance", Math.toRadians(2.0)).degrees();

    private final RobotState state;
    private boolean ready;
    private boolean aimed;

    public ShooterComp(RobotState state, FlywheelConstants flywheelConstants, HoodConstants hoodConstants,
            HopperConstants hopperConstants, KickerConstants kickerConstants) {
        this.state = state;
        flywheel = new Flywheel(state, flywheelConstants);
        hood = new Hood(state, hoodConstants);
        hopper = new Hopper(hopperConstants, flywheel::isReady);
        kicker = new Kicker(kickerConstants, flywheel::isReady);

        addChildSubsystem(hood);
        addChildSubsystem(flywheel);
        addChildSubsystem(hopper);
        addChildSubsystem(kicker);

        allowAllTransitions();
        registerStateCommands();
        registerGauges();

        state.getShooterConstants().tune();

        enable();
    }

    protected void registerStateCommands() {
        registerStateCommand(State.IDLE,
                () -> request(Flywheel.State.IDLE, Hood.State.IDLE, Hopper.State.IDLE, Kicker.State.IDLE));
        registerStateCommand(State.HUB_TRACKING,
                () -> request(Flywheel.State.TRACKING, Hood.State.IDLE, Hopper.State.IDLE, Kicker.State.IDLE));
        registerStateCommand(State.PASS_TRACKING,
                () -> request(Flywheel.State.TRACKING, Hood.State.IDLE, Hopper.State.IDLE, Kicker.State.IDLE));
        registerStateCommand(State.SHOOTING,
                () -> request(Flywheel.State.SHOOT, Hood.State.HUB_TRACKING, Hopper.State.SHOOT, Kicker.State.SHOOT));
        registerStateCommand(State.PASSING,
                () -> request(Flywheel.State.PASS, Hood.State.PASS_TRACKING, Hopper.State.SHOOT, Kicker.State.SHOOT));
        registerStateCommand(State.OUTTAKE,
                () -> request(Flywheel.State.IDLE, Hood.State.IDLE, Hopper.State.OUTAKE, Kicker.State.OUTAKE));
        registerStateCommand(State.TUNING,
                () -> request(Flywheel.State.TUNING, Hood.State.TUNING, Hopper.State.IDLE, Kicker.State.IDLE));
    }

    private void registerGauges() {
        Cockpit.gauge("flywheel", "Flywheel", "rps", flywheel::getSpeed, flywheel::getGoal, flywheel::isReady, this::isWorking);
        Cockpit.gauge("hood", "Hood", "°", () -> Math.toDegrees(hood.getAngle()), () -> Math.toDegrees(hood.getGoal()),
                hood::isAtGoal, this::isWorking);
        Cockpit.gauge("aim", "Aim", "°", () -> Math.toDegrees(Math.abs(state.getCurrentHubSetpoint().getAzimuthRadians())),
                () -> 0.0, () -> aimed, this::isWorking);
        Cockpit.gauge("multiplier", "Multiplier", "×", flywheel::getMultiplier, () -> Math.abs(flywheel.getMultiplier() - 1.0) > 1e-6);
    }

    private boolean isWorking() {
        return getState() != State.IDLE && getState() != State.UNDETERMINED;
    }

    protected void request(Flywheel.State flywheelState, Hood.State hoodState, Hopper.State hopperState,
            Kicker.State kickerState) {
        flywheel.requestTransition(flywheelState);
        hood.requestTransition(hoodState);
        hopper.requestTransition(hopperState);
        kicker.requestTransition(kickerState);
    }

    @Override
    protected void update() {
        boolean flywheelReady = flywheel.isReady();
        boolean hoodReady = hood.isAtGoal();
        aimed = Math.abs(state.getCurrentHubSetpoint().getAzimuthRadians()) < kAimToleranceRadians.get();
        ready = flywheelReady && hoodReady && aimed;
        Logger.recordOutput("Shooter/Ready/Flywheel", flywheelReady);
        Logger.recordOutput("Shooter/Ready/Hood", hoodReady);
        Logger.recordOutput("Shooter/Ready/Aim", aimed);
        Logger.recordOutput("Shooter/Ready/All", ready);
        if (getState() == State.IDLE || getState() == State.UNDETERMINED) {
            return;
        }
        ShooterSetpoint setpoint = isPassing() ? state.getCurrentPassSetpoint() : state.getCurrentHubSetpoint();
        Logger.recordOutput("Shooter/Setpoint/Speed", setpoint.getShooterRPS());
        Logger.recordOutput("Shooter/Setpoint/HoodAngle", setpoint.getHoodRadians());
        Logger.recordOutput("Shooter/Setpoint/AimError", setpoint.getAzimuthRadians());
    }

    @Override
    public void zero() {
        hood.zero();
    }

    @Override
    public void resetMultiplier() {
        flywheel.setMultiplier(1.0);
    }

    @Override
    public void adjustMultiplier(double step) {
        flywheel.setMultiplier(flywheel.getMultiplier() + step);
    }

    @Override
    public boolean isReadyToShoot() {
        return ready;
    }

    @Override
    public boolean isHoldingShot() {
        return flywheel.isOverridden();
    }

    @Override
    public Command spinUp() {
        return flywheel.transitionCommand(Flywheel.State.SHOOT);
    }

    @Override
    public void holdShot(ShooterSetpoint setpoint, boolean spinFlywheel) {
        flywheel.setOverride(() -> flywheel.spin(spinFlywheel ? setpoint.getShooterRPS() : 0.0));
        hood.setOverride(() -> hood.aim(setpoint));
    }

    @Override
    public void holdShot(double speed, double hoodPosition, double hoodFeedforward) {
        flywheel.setOverride(() -> flywheel.spin(speed));
        hood.setOverride(() -> hood.setPos(hoodPosition, hoodFeedforward));
    }

    @Override
    public void releaseShot() {
        flywheel.clearOverride();
        hood.clearOverride();
    }

    @Override
    public void stopFeed() {
        hopper.setOverride(Hopper.State.IDLE);
        kicker.setOverride(Kicker.State.IDLE);
    }

    @Override
    public void reverseFeed() {
        hopper.setOverride(Hopper.State.OUTAKE);
        kicker.setOverride(Kicker.State.OUTAKE);
    }

    @Override
    public void forceFeed() {
        hopper.setOverride(hopper::feed);
        kicker.setOverride(kicker::feed);
    }

    @Override
    public void releaseFeed() {
        hopper.clearOverride();
        kicker.clearOverride();
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
