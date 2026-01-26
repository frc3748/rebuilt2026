package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommands {

    public static PathPlannerAuto getRawAutoWithCommand(Command command) {
        return new PathPlannerAuto(command.getName());
    }

    public static Command getAuto(String autoName) {
        return new PathPlannerAuto(autoName).withName(autoName);
    }

    public static Command testingAuto() {
        return getAuto("Auto Testing");
    }
}
