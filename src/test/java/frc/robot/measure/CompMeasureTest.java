package frc.robot.measure;

import org.junit.jupiter.api.BeforeAll;

import frc.robot.robots.comp.CompRobot;

class CompMeasureTest extends MeasureAutosTest {
    @BeforeAll
    static void setup() {
        boot(new CompRobot());
    }
}
