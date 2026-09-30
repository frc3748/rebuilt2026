package frc.robot.robots.practice;

import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.DriveConfig;

public class PracticeRobot extends RobotDefinition {
    @Override
    public String name() {
        return "Practice";
    }

    @Override
    public DriveConfig drive() {
        return new PracticeDrive();
    }
}
