package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.TunableNumber;

public class DriveSlew {
    private static final double kLoopSeconds = 0.02;

    private final TunableNumber acceleration;
    private final TunableNumber turnAcceleration;
    private double speed;
    private double turnSpeed;

    public DriveSlew(DriveConfig config) {
        acceleration = TunableNumber.field("Drive/Teleop Acceleration", config, "teleopAcceleration");
        turnAcceleration = TunableNumber.field("Drive/Teleop Turn Acceleration", config, "teleopTurnAcceleration");
    }

    public void reset(ChassisSpeeds speeds) {
        speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        turnSpeed = Math.abs(speeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds limit(ChassisSpeeds desired, boolean limitTurning) {
        Translation2d goal = new Translation2d(desired.vxMetersPerSecond, desired.vyMetersPerSecond);
        double wanted = goal.getNorm();
        speed = Math.min(wanted, speed + acceleration.get() * kLoopSeconds);
        Translation2d velocity = wanted > 1e-9 ? goal.times(speed / wanted) : Translation2d.kZero;
        double omega = desired.omegaRadiansPerSecond;
        double wantedTurn = Math.abs(omega);
        turnSpeed = limitTurning ? Math.min(wantedTurn, turnSpeed + turnAcceleration.get() * kLoopSeconds) : wantedTurn;
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), Math.copySign(turnSpeed, omega));
    }
}
