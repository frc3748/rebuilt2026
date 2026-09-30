package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.motor.Motor;
import frc.robot.util.state.StateMachine;

public class Flywheel extends StateMachine<Flywheel.State> {
    private final RobotState state;
    private final Motor flywheel = new Motor(FlywheelConstants.kFlywheel);
    private double rpsMultiplier = 1.0;
    private Runnable override;

    public Flywheel(RobotState state) {
        super("Flywheel", State.UNDETERMINED, State.class);
        this.state = state;

        addOmniTransitions(State.IDLE, State.SHOOT, State.PASS, State.UNDETERMINED, State.TRACKING, State.TUNING);

        Logger.recordOutput("Flywheel/Multiplier", rpsMultiplier);
        SmartDashboard.putData("Flywheel/Reset Multiplier", Commands.runOnce(() -> setMultiplier(1.0))
                .ignoringDisable(true)
                .withName("Reset Multiplier"));
        enable();
    }

    @Override
    protected void update() {
        flywheel.update();

        if (override != null) {
            override.run();
        } else {
            switch (getState()) {
                case SHOOT -> spin(state.getCurrentHubSetpoint().getShooterRPS() * rpsMultiplier);
                case PASS -> spin(state.getCurrentPassSetpoint().getShooterRPS());
                case TRACKING -> spin(FlywheelConstants.kSlowSpeed);
                case TUNING -> spin(FlywheelConstants.kCustomSetpoint.get());
                default -> spin(0);
            }
        }

        Logger.recordOutput("Flywheel/Ready", isReady());
        Logger.recordOutput("Flywheel/Overriden", override != null);
    }

    public void spin(double rps) {
        flywheel.setVelocity(rps);
    }

    public boolean isReady() {
        double desired = flywheel.getSetpoint();
        return desired >= 1 && flywheel.getVelocity() > desired - FlywheelConstants.kSpeedTolerance.get();
    }

    public void setOverride(Runnable override) {
        this.override = override;
    }

    public void setMultiplier(double multiplier) {
        rpsMultiplier = multiplier;
        Logger.recordOutput("Flywheel/Multiplier", multiplier);
    }

    public double getMultiplier() {
        return rpsMultiplier;
    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        SHOOT,
        PASS,
        TRACKING,
        TUNING
    }
}
