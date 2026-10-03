package frc.robot.util.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.util.motor.AutoTune.Feedback;
import frc.robot.util.motor.AutoTune.Gravity;
import frc.robot.util.motor.AutoTune.Model;
import frc.robot.util.motor.AutoTune.Sample;

class AutoTuneTest {
    private static final double kDt = 0.02;

    private static List<Sample> drive(double kS, double kV, double kA, double kG, Gravity gravity, double unitsPerRotation, double[] volts) {
        List<Sample> samples = new ArrayList<>();
        double position = 0.0;
        double velocity = 0.0;
        for (int i = 0; i < volts.length; i++) {
            samples.add(new Sample(i * kDt, volts[i], position, velocity, 1.0));
            double load = gravity == Gravity.NONE ? 0.0 : gravity == Gravity.CONSTANT ? kG : kG * Math.cos(position / unitsPerRotation * 2.0 * Math.PI);
            for (int k = 0; k < 20; k++) {
                double acceleration = (volts[i] - kS * Math.signum(velocity) - kV * velocity - load) / kA;
                velocity += acceleration * kDt / 20.0;
                position += velocity * kDt / 20.0;
            }
        }
        return samples;
    }

    private static double[] ramps() {
        double[] volts = new double[400];
        for (int i = 0; i < volts.length; i++) {
            double t = i * kDt;
            volts[i] = t < 3.0 ? 0.5 + 1.5 * t : t < 4.0 ? 0.0 : t < 5.0 ? 4.0 : t < 6.0 ? -0.5 - 1.5 * (t - 5.0) : -3.0;
        }
        return volts;
    }

    @Test
    void recoversASpinningMechanism() {
        Model model = AutoTune.fit(drive(0.2, 0.38, 0.05, 0.0, Gravity.NONE, 1.0, ramps()), Gravity.NONE, 1.0, 0.1, 100.0).orElseThrow();
        assertEquals(0.2, model.kS(), 0.02);
        assertEquals(0.38, model.kV(), 0.01);
        assertEquals(0.05, model.kA(), 0.005);
    }

    @Test
    void recoversGravityOnAnArm() {
        Model constant = AutoTune.fit(drive(0.15, 0.7, 0.07, 0.4, Gravity.CONSTANT, 1.0, ramps()), Gravity.CONSTANT, 1.0, 0.05, 100.0).orElseThrow();
        assertEquals(0.4, constant.kG(), 0.03);
        Model cosine = AutoTune.fit(drive(0.17, 0.02, 0.002, 0.24, Gravity.COSINE, 360.0, ramps()), Gravity.COSINE, 360.0, 1.0, 100.0).orElseThrow();
        assertEquals(0.24, cosine.kG(), 0.03);
        assertEquals(0.02, cosine.kV(), 0.002);
    }

    @Test
    void startingGainsComeFromTheMeasuredModel() {
        Model model = new Model(0.1, 0.7, 0.07, 0.4, 0.0, 100);
        Feedback position = AutoTune.startingPosition(model, 10.0, 0.8, true);
        assertEquals(0.07 * 100.0, position.kP(), 1e-9);
        assertEquals(2.0 * 0.8 * 10.0 * 0.07 - 0.7, position.kD(), 1e-9);
        assertEquals(0.0, AutoTune.startingPosition(model, 10.0, 0.8, false).kD());
        Feedback spark = AutoTune.forSpark(position);
        assertEquals(position.kP() / 12.0, spark.kP(), 1e-12);
        assertEquals(position.kD() / 12.0 / 0.001, spark.kD(), 1e-9);
        Feedback velocity = AutoTune.startingVelocity(new Model(0.1, 0.4, 0.2, 0.0, 0.0, 100), 0.02);
        assertTrue(velocity.kP() > 0.0);
        assertTrue(AutoTune.startingVelocity(new Model(0.1, 0.4, 0.2, 0.0, 0.0, 100), 0.08).kP() < velocity.kP(), "more delay means softer gains");
        assertEquals(1.5 * position.kP(), AutoTune.scaled(position, 1.5).kP(), 1e-12);
    }

    @Test
    void currentLimitIsNeverMissing() {
        assertEquals(40, MotorConfig.safeCurrentLimit(0));
        assertEquals(40, MotorConfig.safeCurrentLimit(-5));
        assertEquals(35, MotorConfig.safeCurrentLimit(35));
        assertEquals(40, new MotorConfig("Test", 1, MotorConfig.Controller.SPARK_MAX).currentLimit(0).currentLimit);
        assertEquals(40, new MotorConfig("Test", 1, MotorConfig.Controller.SPARK_MAX).currentLimit);
    }
}
