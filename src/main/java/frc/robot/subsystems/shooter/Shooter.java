package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.game.ShooterSetpoint;
import frc.robot.util.state.StateMachine;

public abstract class Shooter extends StateMachine<Shooter.State> {
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

    protected Shooter() {
        super("Shooter", State.UNDETERMINED, State.class);
    }

    public abstract Command spinUp();

    public abstract void holdShot(ShooterSetpoint setpoint, boolean spinFlywheel);

    public abstract void holdShot(double speed, double hoodPosition, double hoodFeedforward);

    public abstract void releaseShot();

    public abstract void stopFeed();

    public abstract void reverseFeed();

    public abstract void forceFeed();

    public abstract void releaseFeed();

    public void zero() {}

    public void resetMultiplier() {}

    public void adjustMultiplier(double step) {}

    public boolean isReadyToShoot() {
        return false;
    }

    public boolean isHoldingShot() {
        return false;
    }

    public boolean isFiring() {
        return getState() == State.SHOOTING || getState() == State.PASSING;
    }

    public boolean isPassing() {
        return getState() == State.PASSING || getState() == State.PASS_TRACKING;
    }
}
