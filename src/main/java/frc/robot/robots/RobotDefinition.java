package frc.robot.robots;

import frc.robot.RobotState;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.vision.CameraConfig;

public interface RobotDefinition {
    String name();

    DriveConfig drive();

    CameraConfig[] cameras();

    default Superstructure createSuperstructure(RobotState state) {
        return new Superstructure(state);
    }
}
