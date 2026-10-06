package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import frc.robot.util.tuning.Source;
import frc.robot.util.tuning.Tuning;

public class TunableNumber {
    private static final List<TunableNumber> all = new ArrayList<>();

    private final Tuning.Value value;
    private final List<DoubleConsumer> listeners = new ArrayList<>();
    private double lastSeen;

    public TunableNumber(String key, double defaultValue) {
        this(key, defaultValue, Source.caller(TunableNumber.class, "<init>", "TunableNumber", 1));
    }

    public TunableNumber(String key, double defaultValue, Source source) {
        value = Tuning.value(key, defaultValue, source);
        lastSeen = defaultValue;
        all.add(this);
    }

    public static TunableNumber field(String key, Object owner, String field) {
        return new TunableNumber(key, Source.read(owner, field), Source.field(owner, field));
    }

    public double get() {
        return value.get();
    }

    public TunableNumber onChange(DoubleConsumer listener) {
        listeners.add(listener);
        return this;
    }

    public TunableNumber integer() {
        value.integer();
        return this;
    }

    public TunableNumber restartToApply() {
        value.restartToApply();
        return this;
    }

    public TunableNumber degrees() {
        value.display("°", Math.toDegrees(1.0));
        return this;
    }

    public static void pollAll() {
        Tuning.periodic();
        for (TunableNumber tunable : all) {
            tunable.poll();
        }
    }

    private void poll() {
        double current = get();
        if (current != lastSeen) {
            lastSeen = current;
            listeners.forEach(listener -> listener.accept(current));
        }
    }
}
