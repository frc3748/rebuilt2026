package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.game.FuelSimulation;
import frc.robot.game.ShooterSetpoint;
import frc.robot.game.ShotVisualizer;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.state.StateMachine;

public class Superstructure {
    private static final int kStartingSimFuel = 8;

    private final RobotState state;
    private final List<StateMachine<?>> subsystems = new ArrayList<>();
    private final ShotVisualizer shotVisualizer;
    private final Timer shotTimer = new Timer();
    private Optional<Intake> intake = Optional.empty();
    private Optional<Shooter> shooter = Optional.empty();
    private FuelSimulation fuel;

    public Superstructure(RobotState state) {
        this.state = state;
        shotVisualizer = new ShotVisualizer(state);
        shotTimer.start();
    }

    public Superstructure withShooter(Shooter shooter) {
        this.shooter = Optional.of(shooter);
        subsystems.add(shooter);
        startFuelSimulation();
        return this;
    }

    public Superstructure withIntake(Intake intake) {
        this.intake = Optional.of(intake);
        subsystems.add(intake);
        startFuelSimulation();
        return this;
    }

    private void startFuelSimulation() {
        if (fuel == null && Constants.kMode == Mode.SIM) {
            fuel = createFuelSimulation();
        }
    }

    private FuelSimulation createFuelSimulation() {
        DriveConfig drive = state.getDrive().getConfig();
        double halfWidth = (drive.trackWidth + Units.inchesToMeters(6)) / 2.0;
        return new FuelSimulation(
                new FuelSimulation.Robot(drive.trackWidth, drive.wheelBase, drive.bumperHeight, ShooterConstants.kShooterToRobotCenter.getZ()),
                new FuelSimulation.Intake(halfWidth, halfWidth + Units.inchesToMeters(8.5), -halfWidth, halfWidth),
                kStartingSimFuel,
                () -> state.getLatestFieldToRobot().getValue().transformBy(new Transform2d(
                        ShooterConstants.kShooterToRobotCenter.getTranslation().toTranslation2d(), Rotation2d.kZero)),
                state::getLatestDesiredFieldRelativeChassisSpeed,
                () -> intake.map(intake -> intake.getState() == Intake.State.INTAKE).orElse(false));
    }

    public Optional<Intake> getIntake() {
        return intake;
    }

    public Optional<Shooter> getShooter() {
        return shooter;
    }

    public Command intakeCommand(Function<Intake, Command> command) {
        return intake.map(command).orElseGet(Commands::none);
    }

    public Command shooterCommand(Function<Shooter, Command> command) {
        return shooter.map(command).orElseGet(Commands::none);
    }

    public Runnable intakeAction(Consumer<Intake> action) {
        return () -> intake.ifPresent(action);
    }

    public Runnable shooterAction(Consumer<Shooter> action) {
        return () -> shooter.ifPresent(action);
    }

    public void clearOverrides() {
        shooter.ifPresent(shooter -> {
            shooter.releaseFeed();
            shooter.releaseShot();
        });
        intake.ifPresent(Intake::clearOverride);
    }

    public List<StateMachine<?>> subsystems() {
        return subsystems;
    }

    public void simulationPeriodic() {
        if (fuel == null) {
            return;
        }
        shooter.ifPresent(this::simulateShots);
        fuel.update();
    }

    private void simulateShots(Shooter shooter) {
        ShooterSetpoint setpoint = shooter.isPassing() ? state.getCurrentPassSetpoint() : state.getCurrentHubSetpoint();
        LinearVelocity exitVelocity = MetersPerSecond.of(setpoint.getShooterRPS());
        Angle launchAngle = Degrees.of(90).minus(Radians.of(setpoint.getHoodRadians()));
        shotVisualizer.update(exitVelocity, launchAngle);

        if (shooter.isFiring() && shotTimer.hasElapsed(ShooterConstants.kSimSecondsBetweenShots)
                && fuel.launch(exitVelocity, launchAngle)) {
            shotTimer.reset();
        }
    }
}
