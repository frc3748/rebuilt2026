package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import frc.robot.RobotState;
import frc.robot.util.TunableNumber;
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
    private final FlywheelConstants constants;
    private final SpinMotor flywheel;
    private final TunableNumber speedTolerance;
    private final TunableNumber customSetpoint;
    private final TunableNumber slowSpeed;
    private double multiplier = 1.0;

    public Flywheel(RobotState robotState, FlywheelConstants constants) {
        super("Flywheel", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        this.constants = constants;
        double metersPerRotation = constants.metersPerRotation();
        flywheel = new SpinMotor(constants.motor.conversion(metersPerRotation, metersPerRotation / 60.0));
        speedTolerance = TunableNumber.field("Flywheel/Speed Tolerance", constants, "speedTolerance");
        customSetpoint = TunableNumber.field("Flywheel/Custom Setpoint", constants, "customSetpoint");
        slowSpeed = TunableNumber.field("Flywheel/Tracking Speed", constants, "slowSpeed");
        addHardware(flywheel);
        allowAllTransitions();

        Logger.recordOutput("Flywheel/Multiplier", multiplier);
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case SHOOT -> spin(robotState.getCurrentHubSetpoint().getShooterRPS() * multiplier);
            case PASS -> spin(robotState.getCurrentPassSetpoint().getShooterRPS());
            case TRACKING -> spin(slowSpeed.get());
            case TUNING -> spin(customSetpoint.get());
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

    public double getSpeed() {
        return flywheel.getVelocity();
    }

    public double getGoal() {
        return flywheel.getGoal();
    }

    public boolean isReady() {
        double goal = flywheel.getGoal();
        return goal >= 1 && flywheel.getVelocity() > goal - speedTolerance.get();
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
