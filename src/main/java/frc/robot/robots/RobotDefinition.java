package frc.robot.robots;

import java.util.List;

import frc.robot.Controls;
import frc.robot.RobotState;
import frc.robot.Superstructure;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.CameraConfig;

public abstract class RobotDefinition {
    public abstract String name();

    public abstract DriveConfig drive();

    public ShooterConstants shooter() {
        return new ShooterConstants();
    }

    public CameraConfig[] cameras() {
        return new CameraConfig[0];
    }

    public Superstructure createSuperstructure(RobotState state) {
        return new Superstructure(state);
    }

    public Controls createControls() {
        return new Controls();
    }

    public List<AutoRoutine> autos(RobotState state) {
        return Autos.all(state);
    }
}
