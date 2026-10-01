package frc.robot.game;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.cockpit.Cockpit;

public class FuelSimulation {
    public record Robot(double widthMeters, double lengthMeters, double bumperHeightMeters, double launchHeightMeters) {}

    public record Intake(double minX, double maxX, double minY, double maxY) {}

    private final FuelSim sim = new FuelSim();
    private final double launchHeightMeters;
    private int heldFuel;
    private int acquired;
    private int launched;

    public FuelSimulation(Robot robot, Intake intake, int startingFuel, Supplier<Pose2d> launcherPose,
            Supplier<ChassisSpeeds> fieldSpeeds, BooleanSupplier intaking) {
        launchHeightMeters = robot.launchHeightMeters();
        heldFuel = startingFuel;

        sim.spawnStartingFuel();
        sim.start();
        sim.enableAirResistance();
        sim.registerRobot(
                Meters.of(robot.widthMeters()),
                Meters.of(robot.lengthMeters()),
                Meters.of(robot.bumperHeightMeters()),
                launcherPose,
                fieldSpeeds);
        sim.registerIntake(intake.minX(), intake.maxX(), intake.minY(), intake.maxY(), intaking, () -> {
            heldFuel++;
            Logger.recordOutput("FuelSim/Acquired", ++acquired);
        });

        Cockpit.button("resetFuel", "Reset fuel", Cockpit.Tab.TEST, Commands.runOnce(() -> {
            sim.clearFuel();
            sim.spawnStartingFuel();
        }).ignoringDisable(true));
    }

    public boolean launch(LinearVelocity velocity, Angle launchAngle) {
        if (heldFuel <= 0) {
            return false;
        }
        heldFuel--;
        Logger.recordOutput("FuelSim/Launched", ++launched);
        sim.launchFuel(velocity, launchAngle, Radians.zero(), Meters.of(launchHeightMeters));
        return true;
    }

    public void update() {
        sim.updateSim();
    }

    public List<Translation3d> positions() {
        return sim.getFuelPositions();
    }
}
