package frc.robot.util.motor;

public class Gains {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public double kG;
    public boolean gravityIsCosine;
    public double maxAccel;
    public double cruiseVel;
    public double allowedError;

    public static Gains of(double kP, double kI, double kD) {
        Gains gains = new Gains();
        gains.kP = kP;
        gains.kI = kI;
        gains.kD = kD;
        return gains;
    }

    public Gains withFeedforward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public Gains copy() {
        Gains copy = of(kP, kI, kD).withFeedforward(kS, kV, kA);
        copy.kG = kG;
        copy.gravityIsCosine = gravityIsCosine;
        copy.maxAccel = maxAccel;
        copy.cruiseVel = cruiseVel;
        copy.allowedError = allowedError;
        return copy;
    }
}
