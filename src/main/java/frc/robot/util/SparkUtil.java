package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import dev.doglog.DogLog;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  public static boolean sparkStickyFault = false;

  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
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
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

  public static void tryUntilOk(SparkMax spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

  public static double getSafe(double[] arr, int index) {
    return (index < arr.length) ? arr[index] : 0.0;
  }

  public static void tunePID(String key, SparkBase motor, SparkBaseConfig defaultConfig, double[] defaults,
      ResetMode resetMode, PersistMode persistMode, boolean feedForward, boolean maxMotion) {
    tunePID(key, motor, defaultConfig, defaults, resetMode, persistMode, feedForward, maxMotion, a -> {
    }, false);
  }

  public static void tunePID(String key, SparkBase motor, SparkBaseConfig defaultConfig, double[] defaults,
      ResetMode resetMode, PersistMode persistMode, boolean feedForward, boolean maxMotion,
      Consumer<Double[]> listener) {
    tunePID(key, motor, defaultConfig, defaults, resetMode, persistMode, feedForward, maxMotion, listener, false);
  }

  public static void tunePID(String key, SparkBase motor, SparkBaseConfig defaultConfig, double[] defaults,
      ResetMode resetMode, PersistMode persistMode, boolean feedForward, boolean maxMotion,
      boolean isCos) {
    tunePID(key, motor, defaultConfig, defaults, resetMode, persistMode, feedForward, maxMotion, a -> {
    }, isCos);
  }

  public static void tunePID(String key, SparkBase motor, SparkBaseConfig defaultConfig, double[] defaults,
      ResetMode resetMode, PersistMode persistMode, boolean feedForward, boolean maxMotion,
      Consumer<Double[]> listener, boolean isCos) {
    double defaultkP = getSafe(defaults, 0);
    double defaultkI = getSafe(defaults, 1);
    double defaultkD = getSafe(defaults, 2);

    double defaultkS = getSafe(defaults, 3);
    double defaultkV = getSafe(defaults, 4);
    double defaultkA = getSafe(defaults, 5);
    double defaultkG = getSafe(defaults, 6);

    double defaultMaxAccel = getSafe(defaults, 7);
    double defaultCruiseVel = getSafe(defaults, 8);
    double defaultDeviationErr = getSafe(defaults, 9);

    Double[] changedDefaults = new Double[10];

    changedDefaults[0] = defaultkP;
    changedDefaults[1] = defaultkI;
    changedDefaults[2] = defaultkD;
    changedDefaults[3] = defaultkS;
    changedDefaults[4] = defaultkV;
    changedDefaults[5] = defaultkA;
    changedDefaults[6] = defaultkG;
    changedDefaults[7] = defaultMaxAccel;
    changedDefaults[8] = defaultCruiseVel;
    changedDefaults[9] = defaultDeviationErr;

    DogLog.tunable(key + "/kP", defaultkP, newP -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.p(newP)), resetMode, persistMode);
      changedDefaults[0] = newP;
      listener.accept(changedDefaults);
    });

    DogLog.tunable(key + "/kI", defaultkI, newI -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.i(newI)), resetMode, persistMode);
      changedDefaults[1] = newI;
      listener.accept(changedDefaults);
    });

    DogLog.tunable(key + "/kD", defaultkD, newD -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.d(newD)), resetMode, persistMode);
      changedDefaults[2] = newD;
      listener.accept(changedDefaults);
    });

    if (feedForward) {
      DogLog.tunable(key + "/kS", defaultkS, newS -> {
        motor.configure(
            defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kS(newS))),
            resetMode, persistMode);
        changedDefaults[3] = newS;
        listener.accept(changedDefaults);
      });

      DogLog.tunable(key + "/kV", defaultkV, newV -> {
        motor.configure(
            defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kV(newV))),
            resetMode, persistMode);
        changedDefaults[4] = newV;
        listener.accept(changedDefaults);
      });

      DogLog.tunable(key + "/kA", defaultkA, newA -> {
        motor.configure(
            defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kA(newA))),
            resetMode, persistMode);
        changedDefaults[5] = newA;
        listener.accept(changedDefaults);
      });

      if (isCos) {
        DogLog.tunable(key + "/kCos", defaultkG, newG -> {
          motor.configure(
              defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kCos(newG))),
              resetMode, persistMode);
          changedDefaults[6] = newG;
          listener.accept(changedDefaults);
        });
      } else {
        DogLog.tunable(key + "/kG", defaultkG, newG -> {
          motor.configure(
              defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kG(newG))),
              resetMode, persistMode);
          changedDefaults[6] = newG;
          listener.accept(changedDefaults);
        });
      }
    }

    if (maxMotion) {
      DogLog.tunable(key + "/kMaxAccel", defaultMaxAccel, newMaxAccel -> {
        defaultConfig
            .apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.maxMotion.maxAcceleration(newMaxAccel)));
        changedDefaults[7] = newMaxAccel;
        listener.accept(changedDefaults);
      });

      DogLog.tunable(key + "/kCruiseVel", defaultCruiseVel, newCruiseVel -> {
        defaultConfig
            .apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.maxMotion.cruiseVelocity(newCruiseVel)));
        changedDefaults[8] = newCruiseVel;
        listener.accept(changedDefaults);
      });

      DogLog.tunable(key + "/kDeviationErr", defaultDeviationErr, newDeviationErr -> {
        defaultConfig.apply(
            defaultConfig.closedLoop.apply(defaultConfig.closedLoop.maxMotion.allowedProfileError(newDeviationErr)));
        changedDefaults[9] = newDeviationErr;
        listener.accept(changedDefaults);
      });
    }
  }
}