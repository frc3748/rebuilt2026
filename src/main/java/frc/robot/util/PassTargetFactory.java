package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.FieldConstants;

public class PassTargetFactory {
    final static double kFarWingX = VisionConstants.kFieldLengthMeters - Units.inchesToMeters(231.2);
    final static double kFarWingPoopBuffer = Units.inchesToMeters(72);

    final static double kLineDrivePoopHeight = Units.inchesToMeters(36.0);

    public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(
                Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).plus(Inches.of(85)), Meter.of(kLineDrivePoopHeight));

    public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(
                Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)), Meter.of(kLineDrivePoopHeight));

    public static Translation3d generate(RobotState robotState) {
        var fieldToRobot = robotState.getLatestFieldToRobot().getValue();

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = fieldToRobot.getMeasureY().gt(FieldConstants.FIELD_WIDTH.div(2));

        Translation3d target = isBlue == onBlueLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;

        if (robotState.isRedAlliance()) {
            target = Util.flipRedBlueXY(target);
        }

        Logger.recordOutput("Poop Pose", target);

        return target;
    }
}