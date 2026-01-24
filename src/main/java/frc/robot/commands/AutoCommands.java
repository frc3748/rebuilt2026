package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoCommands {
    public static PathPlannerAuto getAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public static PathPlannerAuto testingAuto() {
        return getAuto("Auto Testing");
    }
}
