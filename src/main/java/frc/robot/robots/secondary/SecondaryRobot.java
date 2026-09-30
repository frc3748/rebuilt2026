package frc.robot.robots.secondary;

import frc.robot.robots.comp.CompRobot;
import frc.robot.subsystems.drive.DriveConfig;

public class SecondaryRobot extends CompRobot {
    @Override
    public String name() {
        return "Secondary";
    }

    @Override
    public DriveConfig drive() {
        return new SecondaryDrive();
    }
}
