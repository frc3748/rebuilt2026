package frc.robot.util.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.util.TunableNumber;

class TuningTest {
    @BeforeAll
    static void hal() {
        HAL.initialize(500, 0);
    }

    @Test
    void overridesReplaceTheCodeValueAndReachListenersAtStartup() {
        TunableNumber gain = new TunableNumber("Test/Gain", 0.5);
        List<Double> applied = new ArrayList<>();
        gain.onChange(applied::add);
        assertEquals(0.5, gain.get());

        Tuning.override("Test/Gain", 0.7);
        assertEquals(0.7, gain.get());
        TunableNumber.pollAll();
        assertEquals(List.of(0.7), applied);
        TunableNumber.pollAll();
        assertEquals(List.of(0.7), applied);
    }

    @Test
    void overridesRegisteredFirstApplyToLaterTunables() {
        Tuning.override("Test/Early", 4.0);
        TunableNumber early = new TunableNumber("Test/Early", 3.0);
        assertEquals(4.0, early.get());
    }

    @Test
    void sameKeySharesOneValue() {
        TunableNumber first = new TunableNumber("Test/Shared", 1.0);
        TunableNumber second = new TunableNumber("Test/Shared", 9.0);
        assertEquals(first.get(), second.get());
    }

    @Test
    void integersRound() {
        TunableNumber limit = new TunableNumber("Test/Limit", 40.4).integer();
        assertEquals(40.0, limit.get());
    }

    @Test
    void tuningIsOffUntilEnabledAndNothingSavesWhileOff() {
        assertFalse(Tuning.isActive());
        assertEquals(0, Tuning.save());
        Tuning.setEnabled(true);
        assertTrue(Tuning.isActive());
        Tuning.setEnabled(false);
    }
}
