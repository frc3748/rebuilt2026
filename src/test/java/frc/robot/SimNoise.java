package frc.robot;

import java.lang.reflect.Field;
import java.util.Random;

import org.ironmaple.utils.mathutils.MapleCommonMath;

public final class SimNoise {
    private static final long kSeed = 3748;

    private SimNoise() {}

    public static void seed() {
        try {
            Field field = MapleCommonMath.class.getDeclaredField("random");
            field.setAccessible(true);
            ((Random) field.get(null)).setSeed(kSeed);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("maple-sim changed how it makes noise", e);
        }
    }
}
