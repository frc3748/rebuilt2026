package frc.robot.robots.practice;

import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.vision.CameraConfig;

public class PracticeRobot implements RobotDefinition {
    @Override
    public String name() {
        return "Practice";
    }

    @Override
    public DriveConfig drive() {
        return new PracticeDrive();
    }

    @Override
    public CameraConfig[] cameras() {
        return new CameraConfig[0];
    }
}
