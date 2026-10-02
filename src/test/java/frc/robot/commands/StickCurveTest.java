package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class StickCurveTest {
    @Test
    void sticksAreSquared() {
        assertEquals(1.0, DriveCommands.square(1.0), 1e-12);
        assertEquals(0.25, DriveCommands.square(0.5), 1e-12);
        assertEquals(0.01, DriveCommands.square(0.1), 1e-12);
        assertEquals(-0.04, DriveCommands.square(-0.2), 1e-12);
    }

    @Test
    void driftUnderTenPercentIsIgnored() {
        assertEquals(0.0, DriveCommands.square(0.09), 0.0);
        assertEquals(0.0, DriveCommands.square(-0.05), 0.0);
    }
}
