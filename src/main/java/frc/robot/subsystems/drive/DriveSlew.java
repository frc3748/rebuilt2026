package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.TunableNumber;

public class DriveSlew {
    private static final double kLoopSeconds = 0.02;

    private final TunableNumber acceleration;
    private final TunableNumber deceleration;
    private final TunableNumber turnAcceleration;
    private Translation2d velocity = Translation2d.kZero;
    private double omega;

    public DriveSlew(DriveConfig config) {
        acceleration = TunableNumber.field("Drive/Teleop Acceleration", config, "teleopAcceleration");
        deceleration = TunableNumber.field("Drive/Teleop Deceleration", config, "teleopDeceleration");
        turnAcceleration = TunableNumber.field("Drive/Teleop Turn Acceleration", config, "teleopTurnAcceleration");
    }

    public void reset(ChassisSpeeds speeds) {
        velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        omega = speeds.omegaRadiansPerSecond;
    }

    public ChassisSpeeds limit(ChassisSpeeds desired, boolean limitTurning) {
        Translation2d goal = new Translation2d(desired.vxMetersPerSecond, desired.vyMetersPerSecond);
        Translation2d change = goal.minus(velocity);
        double rate = goal.getNorm() < velocity.getNorm() ? deceleration.get() : acceleration.get();
        double step = rate * kLoopSeconds;
        velocity = change.getNorm() <= step ? goal : velocity.plus(change.times(step / change.getNorm()));
        double turnStep = turnAcceleration.get() * kLoopSeconds;
        omega = limitTurning ? omega + Math.max(-turnStep, Math.min(turnStep, desired.omegaRadiansPerSecond - omega))
                : desired.omegaRadiansPerSecond;
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
    }
}
