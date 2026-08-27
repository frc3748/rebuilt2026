package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.SetpointLogAutoLogged;
import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

public class ShooterSetpoint {
    public static Optional<Double> overrideRPS = Optional.empty();

    private double shooterRPS;
    private double shooterStage1RPS = 14.4;
    private double turretRadiansFromCenter = 0;
    private double turretFF = 0;
    private double hoodRadians;
    private double hoodFF;
    private double height;
    private boolean isValid;

    public ShooterSetpoint(double shooterRPS, double turretRadiansFromCenter, double turretFF, double hoodRadians,
            double hoodFF, double height, boolean isValid) {
        this.shooterRPS = shooterRPS;
        this.turretRadiansFromCenter = turretRadiansFromCenter;
        this.turretFF = turretFF;
        this.hoodRadians = hoodRadians;
        this.hoodFF = hoodFF;
        this.height = height;
        this.isValid = isValid;
    }

    public ShooterSetpoint(double shooterRPS, double turretRadiansFromCenter, double turretFF, double hoodRadians,
            double hoodFF, double height) {
        this(shooterRPS, turretRadiansFromCenter, turretFF, hoodRadians, hoodFF, height, true);
    }

    public static void clearOverrideRPS() {
        overrideRPS = Optional.empty();
    }

    public static void setOverrideRPS(double rps) {
        overrideRPS = Optional.of(rps);
    }

    public boolean getIsValid() {
        return this.isValid;
    }

    private static ShooterSetpoint makeSetpoint(RobotState robotState, Rotation2d turretAngle,
            Translation3d turretTarget, double pitchAngleRads, double launchSpeedMetersPerSec) {

        ChassisSpeeds robotSpeeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
        double distanceToTarget = Math.hypot(turretTarget.getX(), turretTarget.getY());
        double hoodFF = -robotSpeeds.vxMetersPerSecond * turretTarget.getZ() /
                (distanceToTarget * distanceToTarget + turretTarget.getZ() * turretTarget.getZ());

        double shooterRPS = launchSpeedMetersPerSec;
        if (overrideRPS.isPresent())
            shooterRPS = overrideRPS.get();
        boolean validSetpoint = true;

        return new ShooterSetpoint(shooterRPS,
                0,
                0,
                pitchAngleRads,
                hoodFF,
                turretTarget.getZ(),
                validSetpoint);
    }

    private static ShooterSetpoint fromTarget(Translation3d target, RobotState robotState,
            boolean useMovingPrediction) {
        Pose2d robotPose = robotState.getLatestFieldToRobot().getValue();
        ChassisSpeeds robotSpeeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();

        TurretCalculator.ShotData shot;
        if (useMovingPrediction) {
            if (RobotState.robotState == 1) {
                shot = TurretCalculator.iterativeMovingShotFromMap(robotState, robotPose, robotSpeeds, target, 10);
            } else {
                shot = TurretCalculator.iterativeMovingShotFromFunnelClearance(robotState, robotPose, robotSpeeds, target, 3);
            }
        } else {
            shot = TurretCalculator.calculateShotFromFunnelClearance(robotState, robotPose, target, target);
        }

        double launchSpeed = shot.getExitVelocity().in(MetersPerSecond);
        double hoodAngle = shot.getHoodAngle().in(Radians);
        Translation3d predictedTarget = shot.getTarget();

        Rotation2d turretAngle = Rotation2d.fromRadians(
                TurretCalculator.calculateAzimuthAngle(robotState, robotPose, predictedTarget,
                        Radians.of(0.0)).in(Radians));

        return makeSetpoint(robotState, turretAngle, predictedTarget, hoodAngle, launchSpeed);
    }

    public static Supplier<ShooterSetpoint> passSetpointSupplier(RobotState robotState) {
        return () -> fromTarget(PassTargetFactory.generate(robotState), robotState, true);
    }

    public static Supplier<ShooterSetpoint> speakerSetpointSupplier(RobotState robotState) {
        return () -> fromTarget(BallTargetFactory.generate(robotState), robotState, true);
    }

    public double getShooterRPS() {
        return shooterRPS;
    }

    public double getShooterStage1RPS() {
        return shooterStage1RPS;
    }

    public double getTurretRadiansFromCenter() {
        return turretRadiansFromCenter;
    }

    public double getTurretFF() {
        return turretFF;
    }

    public double getHoodRadians() {
        return hoodRadians;
    }

    public double getHoodFF() {
        return hoodFF;
    }

    public double getHeight() {
        return height;
    }

    public static SetpointLogAutoLogged getLog(ShooterSetpoint setpoint) {
        SetpointLogAutoLogged log = new SetpointLogAutoLogged();
        log.height = setpoint.getHeight();
        log.shooterRPS = setpoint.getShooterRPS();
        log.turretRadiansFromCenter = 0;
        log.turretFF = 0;
        log.hoodRadians = setpoint.getHoodRadians();
        log.hoodFF = setpoint.getHoodFF();
        log.isValid = setpoint.isValid;

        return log;
    }
}
