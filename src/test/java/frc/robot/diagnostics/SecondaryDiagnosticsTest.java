package frc.robot.diagnostics;

import org.junit.jupiter.api.BeforeAll;

import frc.robot.robots.secondary.SecondaryRobot;

class SecondaryDiagnosticsTest extends DiagnosticAutosTest {
    @BeforeAll
    static void setup() {
        boot(new SecondaryRobot());
    }
}
