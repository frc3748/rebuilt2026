package frc.robot.diagnostics;

import org.junit.jupiter.api.BeforeAll;

import frc.robot.robots.comp.CompRobot;

class CompDiagnosticsTest extends DiagnosticAutosTest {
    @BeforeAll
    static void setup() {
        boot(new CompRobot());
    }
}
