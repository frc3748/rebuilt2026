package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.motor.SpinMotor;
import frc.robot.util.state.StateMachine;

public class Flywheel extends StateMachine<Flywheel.State> {
    public enum State {
        UNDETERMINED,
        IDLE,
        SHOOT,
        PASS,
        TRACKING,
        TUNING
    }

    private final RobotState robotState;
    private final SpinMotor flywheel = new SpinMotor(kFlywheel);
    private double multiplier = 1.0;

    public Flywheel(RobotState robotState) {
        super("Flywheel", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        addHardware(flywheel);
        allowAllTransitions();

        Logger.recordOutput("Flywheel/Multiplier", multiplier);
        SmartDashboard.putData("Flywheel/Reset Multiplier", Commands.runOnce(() -> setMultiplier(1.0))
                .ignoringDisable(true)
                .withName("Reset Multiplier"));
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case SHOOT -> spin(robotState.getCurrentHubSetpoint().getShooterRPS() * multiplier);
            case PASS -> spin(robotState.getCurrentPassSetpoint().getShooterRPS());
            case TRACKING -> spin(kSlowSpeed);
            case TUNING -> spin(kCustomSetpoint.get());
            case IDLE, UNDETERMINED -> spin(0);
        }
    }

    @Override
    protected void update() {
        Logger.recordOutput("Flywheel/Ready", isReady());
    }

    public void spin(double speed) {
        flywheel.set(speed);
    }

    public boolean isReady() {
        double goal = flywheel.getGoal();
        return goal >= 1 && flywheel.getVelocity() > goal - kSpeedTolerance.get();
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
        Logger.recordOutput("Flywheel/Multiplier", multiplier);
    }

    public double getMultiplier() {
        return multiplier;
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
