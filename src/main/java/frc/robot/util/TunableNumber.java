package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.DriverStation;

public class TunableNumber {
    private static final List<TunableNumber> all = new ArrayList<>();

    private final String key;
    private final LoggedNetworkNumber value;
    private final double defaultValue;
    private final List<DoubleConsumer> listeners = new ArrayList<>();
    private double lastSeen;
    private boolean defaultLogged;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        value = new LoggedNetworkNumber("/Tunable/" + key, defaultValue);
        this.defaultValue = defaultValue;
        lastSeen = defaultValue;
        all.add(this);
    }

    public double get() {
        return DriverStation.isFMSAttached() ? defaultValue : value.get();
    }

    public TunableNumber onChange(DoubleConsumer listener) {
        listeners.add(listener);
        return this;
    }

    public static void pollAll() {
        for (TunableNumber tunable : all) {
            tunable.poll();
        }
    }

    private void poll() {
        if (!defaultLogged) {
            Logger.recordOutput("TunableDefaults/" + key, defaultValue);
            defaultLogged = true;
        }
        double current = get();
        if (current != lastSeen) {
            lastSeen = current;
            listeners.forEach(listener -> listener.accept(current));
        }
    }
}
