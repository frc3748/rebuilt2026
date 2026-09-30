package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.vision.VisionConstants;

public class ShotCalculator {
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        Translation2d shooter = robot
                .transformBy(new Transform2d(
                        VisionConstants.kShooterToRobotCenter.getTranslation().toTranslation2d(),
                        new Rotation2d()))
                .getTranslation();
        Distance distance = Meters.of(shooter.getDistance(target.toTranslation2d()));
        Logger.recordOutput("Shooter/DistanceToTarget", distance.in(Meters));
        return distance;
    }

    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = GetTuned.getNumber(
                "Shooter/Gravity InchesPerSec2",
                MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond));
        double vel = velocity.in(InchesPerSecond);
        double xDist = getDistanceToTarget(robot, target).in(Inches);
        double yDist = target.getMeasureZ()
                .minus(VisionConstants.kShooterToRobotCenter.getMeasureZ())
                .in(Inches);

        double angle = Math.atan(
                ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * xDist * xDist + 2 * yDist * vel * vel)))
                        / (g * xDist));
        return Radians.of(angle);
    }

    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double angle = Math.PI / 2 - hoodAngle.in(Radians);
        return Seconds.of(distance.in(Meters) / (exitVelocity.in(MetersPerSecond) * Math.cos(angle)));
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        Translation2d shooter = new Pose3d(robot)
                .transformBy(VisionConstants.kShooterToRobotCenter)
                .toPose2d()
                .getTranslation();
        Translation2d direction = target.toTranslation2d().minus(shooter);
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

    public static ShotData calculateShotFromFunnelClearance(Pose2d robot, Translation3d actualTarget,
            Translation3d predictedTarget) {
        double xDist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double yDist = predictedTarget
                .getMeasureZ()
                .minus(VisionConstants.kShooterToRobotCenter.getMeasureZ())
                .in(Inches);

        double g = GetTuned.getNumber("Shooter/Gravity Funnel InchesPerSec2", 386);
        double funnelRadius = GetTuned.getNumber(
                "Shooter/FunnelRadiusIn",
                VisionConstants.FieldConstants.FUNNEL_RADIUS.in(Inches));
        double funnelHeight = GetTuned.getNumber(
                "Shooter/FunnelHeightIn",
                VisionConstants.FieldConstants.FUNNEL_HEIGHT
                        .plus(ShooterConstants.kDistanceAboveFunnel)
                        .in(Inches));

        double r = funnelRadius * xDist / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = funnelHeight;

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

        return new ShotData(
                linearToAngularVelocity(InchesPerSecond.of(v0), FlywheelConstants.kFlywheelRadius),
                Radians.of(Math.PI / 2 - theta),
                predictedTarget);
    }

    public static ShotData iterativeMovingShotFromFunnelClearance(Pose2d robot, ChassisSpeeds fieldSpeeds,
            Translation3d target, int iterations) {
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
                getDistanceToTarget(robot, target));

        int iters = (int) GetTuned.getNumber("Shooter/PredictionIterations", iterations);
        for (int i = 0; i < iters; i++) {
            Translation3d predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
                    getDistanceToTarget(robot, predictedTarget));
        }
        return shot;
    }

    public static ShotData iterativeMovingShotFromMap(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target,
            int iterations) {
        double distance = getDistanceToTarget(robot, target).in(Meters);
        ShotData shot = ShooterConstants.kShotMap.get(distance).withTarget(target);
        Time timeOfFlight = Seconds.of(ShooterConstants.kTimeOfFlightMap.get(distance));

        int iters = (int) GetTuned.getNumber("Shooter/PredictionIterationsMap", iterations);
        for (int i = 0; i < iters; i++) {
            Translation3d predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            distance = getDistanceToTarget(robot, predictedTarget).in(Meters);
            shot = ShooterConstants.kShotMap.get(distance).withTarget(predictedTarget);
            timeOfFlight = Seconds.of(ShooterConstants.kTimeOfFlightMap.get(distance));
        }
        return shot;
    }

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
        }

        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, VisionConstants.FieldConstants.HUB_BLUE);
        }

        public ShotData withTarget(Translation3d newTarget) {
            return new ShotData(exitVelocity, hoodAngle, newTarget);
        }

        public LinearVelocity getExitVelocity() {
            return angularToLinearVelocity(RadiansPerSecond.of(exitVelocity), FlywheelConstants.kFlywheelRadius);
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
