package frc.robot.game;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

public class PassTargetFactory {
    private static final double kLineDriveHeight = Units.inchesToMeters(36.0);

    public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(
            Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).plus(Inches.of(85)), Meter.of(kLineDriveHeight));
    public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(
            Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)), Meter.of(kLineDriveHeight));

    public static Translation3d generate(RobotState robotState) {
        var fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        boolean onLeftSide = fieldToRobot.getMeasureY().gt(FieldConstants.FIELD_WIDTH.div(2));
        boolean red = AllianceFlip.isRed();

        Translation3d target = !red == onLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;
        if (red) {
            target = AllianceFlip.flip(target);
        }

        Logger.recordOutput("PassTarget", target);
        return target;
    }
}
