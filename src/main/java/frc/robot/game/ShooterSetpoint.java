package frc.robot.game;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;

public class ShooterSetpoint {
    private final double shooterRPS;
    private final double azimuthRadians;
    private final double hoodRadians;
    private final double hoodFF;
    private final double height;

    public ShooterSetpoint(double shooterRPS, double azimuthRadians, double hoodRadians, double hoodFF, double height) {
        this.shooterRPS = shooterRPS;
        this.azimuthRadians = azimuthRadians;
        this.hoodRadians = hoodRadians;
        this.hoodFF = hoodFF;
        this.height = height;
    }

    public static Supplier<ShooterSetpoint> hubSetpointSupplier(RobotState state) {
        return () -> fromTarget(BallTargetFactory.generate(state), state);
    }

    public static Supplier<ShooterSetpoint> passSetpointSupplier(RobotState state) {
        return () -> fromTarget(PassTargetFactory.generate(state), state);
    }

    private static ShooterSetpoint fromTarget(Translation3d target, RobotState state) {
        Pose2d robot = state.getLatestFieldToRobot().getValue();
        ChassisSpeeds robotSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();

        ShotCalculator calculator = state.getShotCalculator();
        ShotCalculator.ShotData shot = Constants.kMode == Mode.REAL
                ? calculator.iterativeMovingShotFromMap(robot, robotSpeeds, target)
                : calculator.iterativeMovingShotFromFunnelClearance(robot, robotSpeeds, target);

        Translation3d predictedTarget = shot.getTarget();
        double azimuth = calculator.calculateAzimuthAngle(robot, predictedTarget).in(Radians);

        double distance = Math.hypot(predictedTarget.getX(), predictedTarget.getY());
        double hoodFF = -robotSpeeds.vxMetersPerSecond * predictedTarget.getZ()
                / (distance * distance + predictedTarget.getZ() * predictedTarget.getZ());

        return new ShooterSetpoint(
                shot.getExitVelocity().in(MetersPerSecond),
                azimuth,
                shot.getHoodAngle().in(Radians),
                hoodFF,
                predictedTarget.getZ());
    }

    public double getShooterRPS() {
        return shooterRPS;
    }

    public double getAzimuthRadians() {
        return azimuthRadians;
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
}
