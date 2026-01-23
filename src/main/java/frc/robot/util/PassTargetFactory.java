package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.vision.VisionConstants;

public class PassTargetFactory {

    // All are BLUE primary.
    final static double kCloseWingX = Units.inchesToMeters(231.2);
    final static double kFarWingX = VisionConstants.kFieldLengthMeters - Units.inchesToMeters(231.2);
    final static double kFarWingPoopBuffer = Units.inchesToMeters(72); // popsitive = further away from driver
    final static double kDefaultXSafetyMargin = Units.inchesToMeters(48);
    final static double kFarXSafetyMargin = Units.inchesToMeters(60);
    final static double kPrimaryYOffsetFromAmpWall = Units.inchesToMeters(72);
    final static double kSecondaryYOffsetFromAmpWall = Units.inchesToMeters(20);

    final static double kNominalPoopHeight = FlywheelConstants.kPassMaxApexHeight;
    final static double kLineDrivePoopHeight = Units.inchesToMeters(36.0);

    public static Translation3d primaryForFarZone() {
        // Aim at the wing line plus some margin in X
        final double targetX = kCloseWingX + kFarXSafetyMargin;
        // Aim at the amp wall plus some margin in Y
        final double targetY = VisionConstants.kFieldWidthMeters - kPrimaryYOffsetFromAmpWall;

        return new Translation3d(targetX, targetY, kNominalPoopHeight);
    }

    public static Translation3d generate(RobotState robotState) {
        var fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        double robotX = fieldToRobot.getX();
        double robotY = fieldToRobot.getY();
        Rotation2d robotHeading = fieldToRobot.getRotation();
        if (robotState.isRedAlliance()) {
            // Make robotX BLUE relative.
            robotX = VisionConstants.kFieldLengthMeters - robotX;
            // Make robot heading alliance relative (0 is robot facing away from alliance
            // wall)
            robotHeading = robotHeading.rotateBy(Rotation2d.fromDegrees(180));
        }

        Translation3d target = new Translation3d();
        if (robotX < (kFarWingX + kFarWingPoopBuffer)) {
            target = VisionConstants.kBluePassArea;

            if ((robotY > 5.5 || robotX < 4.0)
                    && !Util.epsilonEquals(robotHeading.rotateBy(Rotation2d.fromDegrees(180).unaryMinus()).getDegrees(),
                            0, 45)) {
                // When close to the near poop point without stage in the way, shoot line
                // drives.
                target = new Translation3d(target.getX(), target.getY(), kLineDrivePoopHeight);
            }
        } else {
            target = primaryForFarZone();
        }

        // Flip X if red alliance. Y does not flip.
        if (robotState.isRedAlliance()) {
            target = Util.flipRedBlue(target);
        }

        Logger.recordOutput("Poop Pose", target);

        return target;
    }
}