package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class GetTuned {
    private static final Map<String, DoubleSubscriber> tuneList = new HashMap<>();

    public static double getNumber(String name, double constant) {
        return tuneList.computeIfAbsent(name,
                k -> DogLog.tunable(k, constant)).getAsDouble();
    }
}
