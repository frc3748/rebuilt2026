package frc.robot.robots.competition;

import static frc.robot.subsystems.shooter.ShooterConstants.kShooterToRobotCenter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.robots.RobotDefinition;
import frc.robot.robots.Superstructure;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.vision.CameraConfig;

public class CompetitionRobot implements RobotDefinition {
    public static final CameraConfig kShooterCamera = new CameraConfig("Shooter Camera", "limelight-turret", CameraConfig.Type.LIMELIGHT_4)
            .robotToCamera(kShooterToRobotCenter.plus(new Transform3d(
                    new Translation3d(Units.inchesToMeters(4.594), Units.inchesToMeters(4.270), Units.inchesToMeters(4.181)),
                    new Rotation3d(0, Units.degreesToRadians(-20.5), 0))))
            .reportedPoseOffset(new Transform2d(Units.inchesToMeters(4.594), Units.inchesToMeters(4.270), Rotation2d.kZero))
            .stdDevFactor(1.3);

    public static final CameraConfig kChassisCamera = new CameraConfig("Chassis Camera", "limelight", CameraConfig.Type.LIMELIGHT_4)
            .robotToCamera(new Transform3d(
                    new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0.75), Units.inchesToMeters(5.75)),
                    new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180))));

    @Override
    public String name() {
        return "Competition";
    }

    @Override
    public DriveConfig drive() {
        return new CompetitionDrive();
    }

    @Override
    public CameraConfig[] cameras() {
        return new CameraConfig[] { kChassisCamera, kShooterCamera };
    }

    @Override
    public Superstructure createSuperstructure(RobotState state) {
        return new CompetitionSuperstructure(state);
    }
}
