package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.util.motor.Gains;

public final class SparkUtil {
    public static boolean sparkStickyFault = false;

    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            if (command.get() == REVLibError.kOk) {
                return;
            }
            sparkStickyFault = true;
        }
    }

    public static void tune(String key, SparkBase spark, SparkBaseConfig config, Gains gains, boolean feedforward,
            boolean maxMotion) {
        Tuner tuner = new Tuner(key, spark, config);
        tuner.add("kP", gains.kP, value -> config.closedLoop.p(value));
        tuner.add("kI", gains.kI, value -> config.closedLoop.i(value));
        tuner.add("kD", gains.kD, value -> config.closedLoop.d(value));
        if (feedforward) {
            tuner.add("kS", gains.kS, value -> config.closedLoop.feedForward.kS(value));
            tuner.add("kV", gains.kV, value -> config.closedLoop.feedForward.kV(value));
            tuner.add("kA", gains.kA, value -> config.closedLoop.feedForward.kA(value));
            if (gains.gravityIsCosine) {
                tuner.add("kCos", gains.kG, value -> config.closedLoop.feedForward.kCos(value));
            } else {
                tuner.add("kG", gains.kG, value -> config.closedLoop.feedForward.kG(value));
            }
        }
        if (maxMotion) {
            tuner.add("kMaxAccel", gains.maxAccel, value -> config.closedLoop.maxMotion.maxAcceleration(value));
            tuner.add("kCruiseVel", gains.cruiseVel, value -> config.closedLoop.maxMotion.cruiseVelocity(value));
            tuner.add("kDeviationErr", gains.allowedError, value -> config.closedLoop.maxMotion.allowedProfileError(value));
        }
    }

    private record Tuner(String key, SparkBase spark, SparkBaseConfig config) {
        void add(String name, double initial, DoubleConsumer edit) {
            new TunableNumber(key + "/" + name, initial).onChange(value -> {
                edit.accept(value);
                spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
        }
    }

    private SparkUtil() {}
}
