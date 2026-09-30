package frc.robot.util.motor;

public class SpinMotor extends Motor {
    public SpinMotor(MotorConfig config) {
        super(config);
    }

    public void set(double velocity) {
        set(velocity, 0.0);
    }

    public void set(double velocity, double feedforwardVolts) {
        request(Mode.VELOCITY, velocity, feedforwardVolts, 0);
    }

    public boolean isAtGoal(double tolerance) {
        return Math.abs(getVelocity() - getGoal()) < tolerance;
    }
}
