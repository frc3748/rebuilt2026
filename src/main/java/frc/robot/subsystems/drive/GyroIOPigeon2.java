package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<AngularVelocity> rollRate;
    private final StatusSignal<AngularVelocity> pitchRate;
    private final StatusSignal<LinearAcceleration> accelX;
    private final StatusSignal<LinearAcceleration> accelY;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2(DriveConfig config) {
        pigeon = new Pigeon2(config.pigeonCanId);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        roll = pigeon.getRoll();
        pitch = pigeon.getPitch();
        rollRate = pigeon.getAngularVelocityXDevice();
        pitchRate = pigeon.getAngularVelocityYDevice();
        accelX = pigeon.getAccelerationX();
        accelY = pigeon.getAccelerationY();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(config.odometryFrequency);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();

        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        StatusSignal<Angle> yawClone = yaw.clone();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> yawClone.refresh().getValueAsDouble());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawRateRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        BaseStatusSignal.refreshAll(rollRate, pitchRate, pitch, roll, accelX, accelY);
        inputs.rollRadians = Units.degreesToRadians(roll.getValueAsDouble());
        inputs.pitchRadians = Units.degreesToRadians(pitch.getValueAsDouble());
        inputs.rollRateRadPerSec = Units.degreesToRadians(rollRate.getValueAsDouble());
        inputs.pitchRateRadPerSec = Units.degreesToRadians(pitchRate.getValueAsDouble());
        inputs.accelXGs = accelX.getValueAsDouble();
        inputs.accelYGs = accelY.getValueAsDouble();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
