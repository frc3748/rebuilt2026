package frc.robot.util.motor;

public class PosMotor extends Motor {
    public PosMotor(MotorConfig config) {
        super(config);
    }

    public void set(double position) {
        set(position, 0.0);
    }

    public void set(double position, double feedforwardVolts) {
        set(position, feedforwardVolts, 0);
    }

    public void set(double position, double feedforwardVolts, int slot) {
        request(Mode.POSITION, position, feedforwardVolts, slot);
    }

    public void resetPosition(double position) {
        setEncoderPosition(position);
    }

    public boolean isAtGoal(double tolerance) {
        return Math.abs(getPosition() - getGoal()) < tolerance;
    }
}
