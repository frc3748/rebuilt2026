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

public class ShooterComp extends Shooter {
    protected final Flywheel flywheel;
    protected final Hood hood;
    protected final Hopper hopper;
    protected final Kicker kicker;

    private final RobotState state;

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

        ShooterConstants shooter = state.getShooterConstants();
        for (double distance : shooter.shotDistances) {
            new TunableNumber("TOF Tuning/" + distance, shooter.timeOfFlightMap.get(distance))
                    .onChange(tof -> shooter.timeOfFlightMap.put(distance, tof));
        }

        enable();
    }

    protected void registerStateCommands() {
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

    protected void request(Flywheel.State flywheelState, Hood.State hoodState, Hopper.State hopperState,
            Kicker.State kickerState) {
        flywheel.requestTransition(flywheelState);
        hood.requestTransition(hoodState);
        hopper.requestTransition(hopperState);
        kicker.requestTransition(kickerState);
    }

    @Override
    protected void update() {
        if (getState() == State.IDLE || getState() == State.UNDETERMINED) {
            return;
        }
        ShooterSetpoint setpoint = isPassing() ? state.getCurrentPassSetpoint() : state.getCurrentHubSetpoint();
        Logger.recordOutput("Shooter/Setpoint/Speed", setpoint.getShooterRPS());
        Logger.recordOutput("Shooter/Setpoint/HoodAngle", setpoint.getHoodRadians());
        Logger.recordOutput("Shooter/Setpoint/AimError", setpoint.getAzimuthRadians());
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
