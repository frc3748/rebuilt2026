package frc.robot.robots.practice;

import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.tuning.Tuning;

public class PracticeRobot extends RobotDefinition {
    @Override
    public String name() {
        return "Practice";
    }

    @Override
    public DriveConfig drive() {
        return new PracticeDrive();
    }

    @Override
    public void tune() {
        super.tune();
        Tuning.override("Drive/Slow Speed", 0.52);
    }
}
