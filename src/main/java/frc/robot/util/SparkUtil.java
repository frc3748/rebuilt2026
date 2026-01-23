// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import dev.doglog.DogLog;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
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

  /** Attempts to run the command until no error is produced. */
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

  public static void tunePID(String key, SparkBase motor, SparkBaseConfig defaultConfig, double[] defaults, ResetMode resetMode, PersistMode persistMode) {
    double defaultkP = getSafe(defaults, 0);
    double defaultkI = getSafe(defaults, 1);
    double defaultkD = getSafe(defaults, 2);
    double defaultkS = getSafe(defaults, 3);
    double defaultkV = getSafe(defaults, 4);
    double defaultkA = getSafe(defaults, 5);
    double defaultkG = getSafe(defaults, 6);

    DogLog.tunable(key + "/kP", defaultkP, newP -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.p(newP)), resetMode, persistMode);
    });

    DogLog.tunable(key + "/kI", defaultkI, newI -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.i(newI)), resetMode, persistMode);
    });

    DogLog.tunable(key + "/kD", defaultkD, newD -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.d(newD)), resetMode, persistMode);
    });

    DogLog.tunable(key + "/kS", defaultkS, newS -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kS(newS))), resetMode, persistMode);
    });

    DogLog.tunable(key + "/kV", defaultkV, newV -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kV(newV))), resetMode, persistMode);
    });

    DogLog.tunable(key + "/kA", defaultkA, newA -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kA(newA))), resetMode, persistMode);
    });

    DogLog.tunable(key + "/kG", defaultkG, newG -> {
      motor.configure(defaultConfig.apply(defaultConfig.closedLoop.apply(defaultConfig.closedLoop.feedForward.kG(newG))), resetMode, persistMode);
    });
  }
}