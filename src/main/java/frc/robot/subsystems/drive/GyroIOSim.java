package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GyroIOSim implements GyroIO {
    private static final double kGravity = 9.80665;
    private static final double kLoopSeconds = 0.02;

    private final SwerveDriveSimulation simulation;
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public GyroIOSim(SwerveDriveSimulation simulation) {
        this.simulation = simulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        GyroSimulation gyro = simulation.getGyroSimulation();
        inputs.connected = true;
        inputs.yawPosition = gyro.getGyroReading();
        inputs.yawRateRadPerSec = gyro.getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.odometryYawTimestamps = DriveSimulation.odometryTimestamps();
        inputs.odometryYawPositions = gyro.getCachedGyroReadings();

        ChassisSpeeds speeds = simulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d accel = new Translation2d(speeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond)
                .div(kLoopSeconds * kGravity)
                .rotateBy(simulation.getSimulatedDriveTrainPose().getRotation().unaryMinus());
        inputs.accelXGs = accel.getX();
        inputs.accelYGs = accel.getY();
        lastSpeeds = speeds;
    }
}
