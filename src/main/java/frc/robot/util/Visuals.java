package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public final class Visuals {
    public static boolean enabled() {
        return Constants.kMode == Mode.SIM;
    }

    public static <T extends StructSerializable> void record(String key, T value) {
        if (enabled()) {
            Logger.recordOutput(key, value);
        }
    }

    @SafeVarargs
    public static <T extends StructSerializable> void record(String key, T... values) {
        if (enabled()) {
            Logger.recordOutput(key, values);
        }
    }

    private Visuals() {}
}
