package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public final class DriveSimulation {
    public static final double kDriveFrictionVolts = 0.1;
    private static final double kTurnFrictionVolts = 0.2;
    private static final double kTurnInertia = 0.03;

    private DriveSimulation() {}

    public static double[] odometryTimestamps() {
        int ticks = SimulatedArena.getSimulationSubTicksIn1Period();
        double dt = SimulatedArena.getSimulationDt().in(Seconds);
        double now = Timer.getFPGATimestamp();
        double[] timestamps = new double[ticks];
        for (int i = 0; i < ticks; i++) {
            timestamps[i] = now - (ticks - 1 - i) * dt;
        }
        return timestamps;
    }

    public static SwerveDriveSimulation create(DriveConfig config, Pose2d start) {
        SwerveModuleSimulationConfig module = new SwerveModuleSimulationConfig(
                config.driveGearbox, config.turnGearbox, config.driveReduction, config.turnReduction,
                Volts.of(kDriveFrictionVolts), Volts.of(kTurnFrictionVolts), Meters.of(config.wheelRadiusMeters),
                KilogramSquareMeters.of(kTurnInertia), config.wheelCOF);
        DriveTrainSimulationConfig drivetrain = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(config.robotMassKg))
                .withTrackLengthTrackWidth(Meters.of(config.wheelBase), Meters.of(config.trackWidth))
                .withBumperSize(Meters.of(config.bumperLength()), Meters.of(config.bumperWidth()))
                .withSwerveModule(module)
                .withGyro(config.gyro == DriveConfig.GyroType.NAVX ? COTS.ofNav2X() : COTS.ofPigeon2());
        SwerveDriveSimulation simulation = new SwerveDriveSimulation(drivetrain, start);
        SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
        return simulation;
    }
}
