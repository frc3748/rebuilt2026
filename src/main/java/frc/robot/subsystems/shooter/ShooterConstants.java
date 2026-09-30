package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.game.ShotCalculator;
import frc.robot.game.ShotCalculator.ShotData;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;

public final class ShooterConstants {
    public static final Transform3d kShooterToRobotCenter = new Transform3d(
            new Translation3d(Units.inchesToMeters(-3.290), Units.inchesToMeters(-4.750), Units.inchesToMeters(13.735 - 0.45)),
            Rotation3d.kZero);
    public static final Distance kDistanceAboveFunnel = Inches.of(20);
    public static final double kTimeOfFlightOffsetSeconds = 0.15;
    public static final double kSimSecondsBetweenShots = 0.08;

    public static final InterpolatingTreeMap<Double, ShotData> kShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);
    public static final InterpolatingDoubleTreeMap kTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    public static final List<Double> kShotDistances = new ArrayList<>();

    static {
        addShot(1.409409, 8.7, 0.0, 0.162001034483);
        addShot(1.822781, 9.8, 0.0, 0.185998061224);
        addShot(2.088259, 9.96, 0.0, 0.209664558233);
        addShot(2.338346, 10.4, 0.0, 0.224840961538);
        addShot(2.527475, 10.8, 0.0, 0.234025462963);
        addShot(2.787548, 10.6, 0.5, 0.299659813031);
        addShot(3.097655, 10.6, 0.54, 0.340711957478);
        addShot(3.59677, 11.15, 0.59, 0.391724169605);
        addShot(3.71836, 10.85, 0.6, 0.415232281986);
        addShot(4.033556, 11.2, 0.65, 0.452388214944);
        addShot(4.611542, 11.7, 0.64, 0.491398794987);
        addShot(5.050931, 12.0, 0.66, 0.532803868044);
        addShot(5.525151, 12.35, 0.68, 0.575355380899);
        addShot(5.944625, 12.9, 0.7, 0.602508139682);
        addShot(6.546274, 13.55, 0.78, 0.679576103937);
    }

    private static void addShot(double distanceMeters, double exitVelocityMetersPerSec, double hoodRadians,
            double timeOfFlightSeconds) {
        kShotMap.put(distanceMeters, new ShotData(
                ShotCalculator.linearToAngularVelocity(
                        MetersPerSecond.of(exitVelocityMetersPerSec), FlywheelConstants.kFlywheelRadius),
                Radians.of(hoodRadians)));
        kTimeOfFlightMap.put(distanceMeters, timeOfFlightSeconds + kTimeOfFlightOffsetSeconds);
        kShotDistances.add(distanceMeters);
    }

    private ShooterConstants() {}
}
