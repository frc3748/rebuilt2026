package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private static final double kLoopSeconds = 0.02;
  private static final String[] kNames = {"FL", "FR", "BL", "BR"};
  public static final String kAlertGroup = "Devices";

  private final double wheelRadiusMeters;
  private final double maxSteerVelocity;
  private Rotation2d lastTarget;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert encoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index, double wheelRadiusMeters, double maxSteerVelocity) {
    this.io = io;
    this.index = index;
    this.wheelRadiusMeters = wheelRadiusMeters;
    this.maxSteerVelocity = maxSteerVelocity;
    driveDisconnectedAlert = new Alert(kAlertGroup, label() + " drive motor disconnected", AlertType.kError);
    turnDisconnectedAlert = new Alert(kAlertGroup, label() + " turn motor disconnected", AlertType.kError);
    encoderDisconnectedAlert = new Alert(kAlertGroup, label() + " turn encoder disconnected", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    encoderDisconnectedAlert.set(!inputs.encoderConnected);
  }

  private String label() {
    return index < kNames.length ? kNames[index] : "Module " + index;
  }

  public List<String> disconnected() {
    List<String> found = new ArrayList<>();
    if (!inputs.driveConnected) {
      found.add(label() + " drive");
    }
    if (!inputs.turnConnected) {
      found.add(label() + " turn");
    }
    if (!inputs.encoderConnected) {
      found.add(label() + " encoder");
    }
    return found;
  }

  public void runSetpoint(SwerveModuleState state, double acceleration) {
    Rotation2d requested = state.angle;
    state.optimize(getAngle());
    if (state.angle.minus(requested).getCos() < 0.0) {
      acceleration = -acceleration;
    }
    double scale = state.angle.minus(inputs.turnPosition).getCos();
    state.cosineScale(inputs.turnPosition);

    io.setDriveVelocity(state.speedMetersPerSecond, acceleration * scale);
    io.setTurnPosition(state.angle, steerVelocity(state.angle));
  }

  private double steerVelocity(Rotation2d target) {
    double velocity = lastTarget == null ? 0.0 : target.minus(lastTarget).getRadians() / kLoopSeconds;
    lastTarget = target;
    return Math.abs(velocity) > maxSteerVelocity ? 0.0 : velocity;
  }

  public void runCharacterization(double output) {
    lastTarget = null;
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero, 0.0);
  }

  public void runAngle(Rotation2d angle) {
    lastTarget = null;
    io.setDriveOpenLoop(0.0);
    io.setTurnPosition(angle, 0.0);
  }

  public void stop() {
    lastTarget = null;
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public void liftDriveCurrentLimit(double amps) {
    io.liftDriveCurrentLimit(amps);
  }

  public void restoreDriveCurrentLimit() {
    io.restoreDriveCurrentLimit();
  }

  public double getDriveCurrentAmps() {
    return inputs.driveCurrentAmps;
  }

  public double getDriveAppliedVolts() {
    return inputs.driveAppliedVolts;
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}