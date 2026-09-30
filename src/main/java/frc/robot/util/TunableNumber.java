package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class TunableNumber {
    private final DoubleSubscriber subscriber;

    public TunableNumber(String key, double defaultValue) {
        subscriber = DogLog.tunable(key, defaultValue);
    }

    public double get() {
        return subscriber.getAsDouble();
    }
}
