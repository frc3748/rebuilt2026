package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public record HeadingSample(double timestamp, Rotation2d heading) {}
