package frc.robot.measure;

import org.junit.jupiter.api.BeforeAll;

import frc.robot.robots.practice.PracticeRobot;

class PracticeMeasureTest extends MeasureAutosTest {
    @BeforeAll
    static void setup() {
        boot(new PracticeRobot());
    }
}
