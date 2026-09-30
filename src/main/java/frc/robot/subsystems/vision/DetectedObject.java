package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;

public record DetectedObject(double timestamp, int classId, Translation2d fieldPosition, double confidence) {}
