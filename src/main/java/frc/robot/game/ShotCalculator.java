package frc.robot.game;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.TunableNumber;

public class ShotCalculator {
    private final ShooterConstants shooter;
    private final TunableNumber funnelGravity = new TunableNumber("Shooter/Gravity Funnel InchesPerSec2", 386);
    private final TunableNumber funnelRadius = new TunableNumber(
            "Shooter/FunnelRadiusIn", FieldConstants.FUNNEL_RADIUS.in(Inches));
    private final TunableNumber funnelHeight;
    private final TunableNumber funnelIterations = new TunableNumber("Shooter/PredictionIterations", 3);
    private final TunableNumber mapIterations = new TunableNumber("Shooter/PredictionIterationsMap", 10);

    public ShotCalculator(ShooterConstants shooter) {
        this.shooter = shooter;
        funnelHeight = new TunableNumber(
                "Shooter/FunnelHeightIn", FieldConstants.FUNNEL_HEIGHT.plus(shooter.distanceAboveFunnel).in(Inches));
    }

    public Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        Translation2d shooterPosition = robot
                .transformBy(new Transform2d(
                        shooter.shooterToRobotCenter.getTranslation().toTranslation2d(),
                        new Rotation2d()))
                .getTranslation();
        Distance distance = Meters.of(shooterPosition.getDistance(target.toTranslation2d()));
        Logger.recordOutput("Shooter/DistanceToTarget", distance.in(Meters));
        return distance;
    }

    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double angle = Math.PI / 2 - hoodAngle.in(Radians);
        return Seconds.of(distance.in(Meters) / (exitVelocity.in(MetersPerSecond) * Math.cos(angle)));
    }

    public Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        Translation2d shooterPosition = new Pose3d(robot)
                .transformBy(shooter.shooterToRobotCenter)
                .toPose2d()
                .getTranslation();
        Translation2d direction = target.toTranslation2d().minus(shooterPosition);
        Rotation2d azimuth = direction.getNorm() > 1e-6
                ? direction.getAngle().minus(robot.getRotation())
                : new Rotation2d();

        Logger.recordOutput("Shooter/DesiredAzimuthRad", azimuth.getRadians());
        return Radians.of(azimuth.getRadians());
    }

    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double seconds = timeOfFlight.in(Seconds);
        return new Translation3d(
                target.getX() - fieldSpeeds.vxMetersPerSecond * seconds,
                target.getY() - fieldSpeeds.vyMetersPerSecond * seconds,
                target.getZ());
    }

    public ShotData calculateShotFromFunnelClearance(Pose2d robot, Translation3d actualTarget,
            Translation3d predictedTarget) {
        double xDist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double yDist = predictedTarget
                .getMeasureZ()
                .minus(shooter.shooterToRobotCenter.getMeasureZ())
                .in(Inches);

        double g = funnelGravity.get();
        double r = funnelRadius.get() * xDist / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = funnelHeight.get();

        double a1 = xDist * xDist;
        double b1 = xDist;
        double d1 = yDist;
        double a2 = -xDist * xDist + (xDist - r) * (xDist - r);
        double b2 = -r;
        double d2 = h;
        double bm = -b2 / b1;
        double a3 = bm * a1 + a2;
        double d3 = bm * d1 + d2;
        double a = d3 / a3;
        double b = (d1 - a1 * a) / b1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * Math.cos(theta) * Math.cos(theta)));

        if (Double.isNaN(v0) || Double.isNaN(theta)) {
            v0 = 0;
            theta = 0;
        }

        return new ShotData(InchesPerSecond.of(v0), Radians.of(Math.PI / 2 - theta), predictedTarget);
    }

    public ShotData iterativeMovingShotFromFunnelClearance(Pose2d robot, ChassisSpeeds fieldSpeeds,
            Translation3d target) {
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
                getDistanceToTarget(robot, target));

        int iters = (int) funnelIterations.get();
        for (int i = 0; i < iters; i++) {
            Translation3d predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
                    getDistanceToTarget(robot, predictedTarget));
        }
        return shot;
    }

    public ShotData iterativeMovingShotFromMap(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target) {
        double distance = getDistanceToTarget(robot, target).in(Meters);
        ShotData shot = shooter.shotMap.get(distance).withTarget(target);
        Time timeOfFlight = Seconds.of(shooter.timeOfFlightMap.get(distance));

        int iters = (int) mapIterations.get();
        for (int i = 0; i < iters; i++) {
            Translation3d predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getDistanceToTarget(robot, predictedTarget).in(Meters);
            shot = shooter.shotMap.get(distance).withTarget(predictedTarget);
            timeOfFlight = Seconds.of(shooter.timeOfFlightMap.get(distance));
        }
        return shot;
    }

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target);
        }

        public ShotData(LinearVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.HUB_BLUE);
        }

        public ShotData withTarget(Translation3d newTarget) {
            return new ShotData(exitVelocity, hoodAngle, newTarget);
        }

        public LinearVelocity getExitVelocity() {
            return MetersPerSecond.of(exitVelocity);
        }

        public Angle getHoodAngle() {
            return Radians.of(hoodAngle);
        }

        public Translation3d getTarget() {
            return target;
        }

        public static ShotData interpolate(ShotData start, ShotData end, double t) {
            return new ShotData(
                    MathUtil.interpolate(start.exitVelocity, end.exitVelocity, t),
                    MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
                    end.target);
        }
    }
}
