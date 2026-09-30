package frc.robot.subsystems.drive;

import java.util.Queue;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
    private final AHRS navX;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX(DriveConfig config) {
        navX = new AHRS(config.navXPort, config.navXUpdateRateHz);
        navX.zeroYaw();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

        inputs.rollRadians = Units.degreesToRadians(navX.getRoll());
        inputs.pitchRadians = Units.degreesToRadians(navX.getPitch());
        inputs.rollRateRadPerSec = Units.degreesToRadians(navX.getRawGyroX());
        inputs.pitchRateRadPerSec = Units.degreesToRadians(navX.getRawGyroY());
        inputs.yawRateRadPerSec = inputs.yawVelocityRadPerSec;
        inputs.accelXGs = navX.getWorldLinearAccelX();
        inputs.accelYGs = navX.getWorldLinearAccelY();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map(value -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
