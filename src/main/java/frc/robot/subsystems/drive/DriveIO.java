package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public interface DriveIO {
    @AutoLog
  public static class DriveIOInputs {
    public Pose2d currentPose = new Pose2d();
    public Pose2d goalPose = new Pose2d();

    public double speedX = 0;
    public double speedY = 0;
    public double speedT = 0;

    public ChassisSpeeds chassieSpeeds = new ChassisSpeeds();
    public SwerveModuleState[] optimizedModStates = new SwerveModuleState[] {};
    public SwerveModuleState[] modStates = new SwerveModuleState[] {};
  }

  public default void updateInputs(DriveIOInputs inputs) {}
}
