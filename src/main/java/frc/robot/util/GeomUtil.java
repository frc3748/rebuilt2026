package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;

public class GeomUtil {
  public static Transform2d translationToTransform(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  public static Transform2d rotationToTransform(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  public static Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  public static Pose2d rotationToPose(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  public static Twist2d multiplyTwist(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  public static Transform3d pose3dToTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  public static Pose3d transform3dToPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  public static Translation2d translation3dTo2dXY(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getY());
  }

  public static Translation2d translation3dTo2dXZ(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }
}