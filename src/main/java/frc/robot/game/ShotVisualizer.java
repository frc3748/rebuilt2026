package frc.robot.game;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotState;

public class ShotVisualizer {
    private static final int kPoints = 50;
    private static final double kStepSeconds = 0.04;

    private final RobotState state;
    private final Translation3d[] trajectory = new Translation3d[kPoints];

    public ShotVisualizer(RobotState state) {
        this.state = state;
    }

    public void update(LinearVelocity exitVelocity, Angle launchAngle) {
        Pose2d robot = state.getLatestFieldToRobot().getValue();
        ChassisSpeeds fieldSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();
        Transform3d shooterToRobotCenter = state.getShooterConstants().shooterToRobotCenter;
        Translation2d shooter = robot
                .transformBy(new Transform2d(
                        shooterToRobotCenter.getTranslation().toTranslation2d(),
                        new Rotation2d()))
                .getTranslation();

        double horizontal = Math.cos(launchAngle.in(Radians)) * exitVelocity.in(MetersPerSecond);
        double vertical = Math.sin(launchAngle.in(Radians)) * exitVelocity.in(MetersPerSecond);
        double xVel = horizontal * robot.getRotation().getCos() + fieldSpeeds.vxMetersPerSecond;
        double yVel = horizontal * robot.getRotation().getSin() + fieldSpeeds.vyMetersPerSecond;
        double z0 = shooterToRobotCenter.getZ();

        for (int i = 0; i < kPoints; i++) {
            double t = i * kStepSeconds;
            trajectory[i] = new Translation3d(
                    shooter.getX() + xVel * t,
                    shooter.getY() + yVel * t,
                    z0 + vertical * t - 0.5 * 9.81 * t * t);
        }

        Logger.recordOutput("Shooter/Trajectory", trajectory);
    }
}
