package frc.robot.robots.competitionv2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robots.competition.CompetitionDrive;

public class CompetitionV2Drive extends CompetitionDrive {
    public CompetitionV2Drive() {
        frontLeft = new ModuleConstants(8, 3, 4, Rotation2d.kZero, false);
        frontRight = new ModuleConstants(4, 7, 2, Rotation2d.kZero, false);
        backLeft = new ModuleConstants(2, 9, 1, Rotation2d.kZero, false);
        backRight = new ModuleConstants(6, 5, 3, Rotation2d.kZero, false);

        wheelRadiusMeters = 0.0508;
        driveReduction = 6.48;
        turnReduction = 12.1;
        turnInverted = true;
        turnEncoderInverted = true;
        maxSpeedMetersPerSec = 5.265648;
    }
}
