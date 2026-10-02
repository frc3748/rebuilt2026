package frc.robot.diagnostics;

import org.junit.jupiter.api.BeforeAll;

import frc.robot.robots.practice.PracticeRobot;

class PracticeDiagnosticsTest extends DiagnosticAutosTest {
    @BeforeAll
    static void setup() {
        boot(new PracticeRobot());
    }
}
