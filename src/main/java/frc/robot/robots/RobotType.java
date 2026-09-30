package frc.robot.robots;

import java.util.function.Supplier;

import frc.robot.robots.comp.CompRobot;
import frc.robot.robots.practice.PracticeRobot;
import frc.robot.robots.secondary.SecondaryRobot;

public enum RobotType {
    COMP(CompRobot::new),
    SECONDARY(SecondaryRobot::new),
    PRACTICE(PracticeRobot::new);

    private final Supplier<RobotDefinition> factory;

    RobotType(Supplier<RobotDefinition> factory) {
        this.factory = factory;
    }

    public RobotDefinition create() {
        return factory.get();
    }
}
