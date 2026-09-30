package frc.robot.robots;

import java.util.function.Supplier;

import frc.robot.robots.competition.CompetitionRobot;
import frc.robot.robots.competitionv2.CompetitionV2Robot;
import frc.robot.robots.practice.PracticeRobot;

public enum RobotType {
    COMPETITION(CompetitionRobot::new),
    COMPETITION_V2(CompetitionV2Robot::new),
    PRACTICE(PracticeRobot::new);

    private final Supplier<RobotDefinition> factory;

    RobotType(Supplier<RobotDefinition> factory) {
        this.factory = factory;
    }

    public RobotDefinition create() {
        return factory.get();
    }
}
