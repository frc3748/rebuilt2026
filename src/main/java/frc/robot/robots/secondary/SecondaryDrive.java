package frc.robot.robots.secondary;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robots.comp.CompDrive;

public class SecondaryDrive extends CompDrive {
    public SecondaryDrive() {
        frontLeft = new ModuleConstants(8, 3, 4, Rotation2d.fromRotations(0.27490234375 + 0.5), false);
        frontRight = new ModuleConstants(4, 7, 2, Rotation2d.fromRotations(0.084716796875), false);
        backLeft = new ModuleConstants(2, 9, 1, Rotation2d.fromRotations(-0.0263671875 + 0.5), false);
        backRight = new ModuleConstants(6, 5, 3, Rotation2d.fromRotations(-0.4609375), false);

        driveReduction = 6.48;
    }
}
