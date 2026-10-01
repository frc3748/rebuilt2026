package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.BuildInfo;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.motor.Motor;
import frc.robot.util.motor.PosMotor;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Level;
import frc.robot.util.state.StateMachine;

public final class SelfTest {
    private static final double kDriveVolts = 1.5;
    private static final double kDriveSeconds = 1.0;
    private static final double kTurnSeconds = 1.0;
    private static final double kSpinVolts = 2.0;
    private static final double kSpinSeconds = 0.8;
    private static final double kMoveSeconds = 1.5;
    private static final double kRestSeconds = 0.4;
    private static final double kMinDriveMetersPerSec = 0.2;
    private static final double kTurnToleranceDegrees = 7.0;
    private static final double kMinSpin = 0.01;
    private static final double kMoveTolerance = 0.15;

    private static final Path kSaved = RobotBase.isReal() ? Path.of("/home/lvuser/selftest.txt") : Path.of("build", "selftest.txt");
    private static final List<String> failures = new ArrayList<>();
    private static boolean passed;

    public record Saved(boolean passed, long epochMillis, String gitSha, List<String> failures) {
        public boolean sameCode() {
            return gitSha.equals(BuildInfo.GIT_SHA);
        }

        public double hoursAgo() {
            return (System.currentTimeMillis() - epochMillis) / 3.6e6;
        }
    }

    private SelfTest() {}

    public static boolean hasPassed() {
        return passed;
    }

    public static List<String> failures() {
        return failures;
    }

    public static Optional<Saved> saved() {
        try {
            String[] parts = Files.readString(kSaved).trim().split("\t", -1);
            List<String> saved = parts.length > 3 && !parts[3].isEmpty() ? Arrays.asList(parts[3].split("\\|")) : List.of();
            return Optional.of(new Saved(Boolean.parseBoolean(parts[0]), Long.parseLong(parts[1]), parts[2], saved));
        } catch (IOException | RuntimeException e) {
            return Optional.empty();
        }
    }

    private static void save() {
        try {
            Files.createDirectories(kSaved.toAbsolutePath().getParent());
            Files.writeString(kSaved, String.join("\t", Boolean.toString(passed), Long.toString(System.currentTimeMillis()),
                    BuildInfo.GIT_SHA, String.join("|", failures)));
        } catch (IOException e) {
            Cockpit.toast(Level.WARNING, "Couldn't save the self-test result", e.getMessage());
        }
    }

    public static Command build(RobotState state) {
        Drive drive = state.getDrive();
        List<StateMachine<?>> machines = new ArrayList<>();
        collectMechanisms(state, machines);

        List<Command> steps = new ArrayList<>();
        steps.add(step("Drive forward", "Drive", "drive", Double.NaN, Commands.sequence(
                Commands.run(() -> drive.runCharacterization(kDriveVolts), drive).withTimeout(kDriveSeconds),
                Commands.runOnce(() -> checkDrive(drive)))
                .finallyDo(drive::stop)));
        steps.add(step("Turn to 90", "Drive", "turn", 90.0, Commands.sequence(
                Commands.run(() -> drive.runModuleAngles(Rotation2d.kCCW_90deg), drive).withTimeout(kTurnSeconds),
                Commands.runOnce(() -> checkTurn(drive, 90.0)))));
        steps.add(step("Turn to 0", "Drive", "turn", 0.0, Commands.sequence(
                Commands.run(() -> drive.runModuleAngles(Rotation2d.kZero), drive).withTimeout(kTurnSeconds),
                Commands.runOnce(() -> checkTurn(drive, 0.0)))
                .finallyDo(drive::stop)));
        for (StateMachine<?> machine : machines) {
            for (Motor motor : machine.getMotors()) {
                steps.add(motorStep(machine, motor));
            }
        }

        return Commands.sequence(
                        Commands.runOnce(() -> {
                            failures.clear();
                            passed = false;
                            machines.forEach(machine -> machine.setOverride(holdStill(machine, null)));
                        }),
                        Commands.sequence(steps.toArray(Command[]::new)),
                        Commands.runOnce(() -> {
                            passed = failures.isEmpty();
                            save();
                            Cockpit.toast(passed ? Level.INFO : Level.ERROR, passed ? "Self-test passed" : "Self-test failed",
                                    passed ? "Every mechanism checked out" : String.join(", ", failures));
                        }))
                .finallyDo(interrupted -> {
                    machines.forEach(StateMachine::clearOverride);
                    record("", "", "", Double.NaN);
                    Logger.recordOutput("SelfTest/Passed", passed);
                    Logger.recordOutput("SelfTest/Failures", failures.toArray(String[]::new));
                    if (interrupted) {
                        Cockpit.toast(Level.WARNING, "Self-test stopped early", "It didn't finish, so it doesn't count");
                    }
                })
                .withName("Self Test");
    }

    private static void collectMechanisms(StateMachine<?> machine, List<StateMachine<?>> machines) {
        if (!machine.getMotors().isEmpty()) {
            machines.add(machine);
        }
        machine.getChildSubsystems().forEach(child -> collectMechanisms(child, machines));
    }

    private static Command motorStep(StateMachine<?> machine, Motor motor) {
        double target = motor.getSelfTestPosition();
        if (motor instanceof PosMotor && !Double.isNaN(target)) {
            double[] start = new double[1];
            return step(motor.getName(), motor.getName(), "move", target, Commands.sequence(
                    Commands.runOnce(() -> {
                        start[0] = motor.getPosition();
                        machine.setOverride(with(holdStill(machine, motor), () -> motor.holdPosition(target)));
                    }),
                    Commands.waitSeconds(kMoveSeconds),
                    Commands.runOnce(() -> {
                        expectNear(motor, target, start[0], "didn't reach its test position");
                        machine.setOverride(with(holdStill(machine, motor), () -> motor.holdPosition(start[0])));
                    }),
                    Commands.waitSeconds(kMoveSeconds),
                    Commands.runOnce(() -> expectNear(motor, start[0], target, "didn't come back"))));
        }
        if (motor instanceof PosMotor) {
            return step(motor.getName(), motor.getName(), "hold", Double.NaN, Commands.waitSeconds(kRestSeconds));
        }
        return step(motor.getName(), motor.getName(), "spin", kSpinVolts, Commands.sequence(
                Commands.runOnce(() -> machine.setOverride(
                        with(holdStill(machine, motor), () -> motor.setVoltage(kSpinVolts)))),
                Commands.waitSeconds(kSpinSeconds),
                Commands.runOnce(() -> {
                    if (!motor.isConnected()) {
                        fail(motor.getName(), "disconnected");
                    } else if (motor.getVelocity() < kMinSpin) {
                        fail(motor.getName(), "didn't spin forward");
                    }
                    machine.setOverride(holdStill(machine, null));
                }),
                Commands.waitSeconds(kRestSeconds)));
    }

    private static void checkDrive(Drive drive) {
        String[] names = {"FL", "FR", "BL", "BR"};
        SwerveModuleState[] states = drive.getModuleStates();
        for (int index = 0; index < states.length; index++) {
            if (states[index].speedMetersPerSecond < kMinDriveMetersPerSec) {
                fail(names[index] + " drive", "didn't drive forward");
            }
        }
    }

    private static void checkTurn(Drive drive, double targetDegrees) {
        String[] names = {"FL", "FR", "BL", "BR"};
        SwerveModuleState[] states = drive.getModuleStates();
        for (int index = 0; index < states.length; index++) {
            double error = Math.abs(MathUtil.inputModulus(states[index].angle.getDegrees() - targetDegrees, -90.0, 90.0));
            if (error > kTurnToleranceDegrees) {
                fail(names[index] + " turn", "didn't reach " + (int) targetDegrees + "°");
            }
        }
    }

    private static void expectNear(Motor motor, double target, double from, String problem) {
        double tolerance = Math.max(Math.abs(target - from) * kMoveTolerance, 1e-3);
        if (!motor.isConnected()) {
            fail(motor.getName(), "disconnected");
        } else if (Math.abs(motor.getPosition() - target) > tolerance) {
            fail(motor.getName(), problem);
        }
    }

    private static void fail(String device, String problem) {
        failures.add(device.replace("Motors/", "") + ": " + problem);
    }

    private static Runnable holdStill(StateMachine<?> machine, Motor active) {
        Map<Motor, Double> positions = new HashMap<>();
        machine.getMotors().forEach(motor -> positions.put(motor, motor.getPosition()));
        return () -> machine.getMotors().stream()
                .filter(motor -> motor != active)
                .forEach(motor -> {
                    if (motor instanceof PosMotor) {
                        motor.holdPosition(positions.get(motor));
                    } else {
                        motor.stop();
                    }
                });
    }

    private static Runnable with(Runnable first, Runnable second) {
        return () -> {
            first.run();
            second.run();
        };
    }

    private static Command step(String name, String device, String kind, double target, Command command) {
        return command.beforeStarting(() -> record(name, device, kind, target));
    }

    private static void record(String name, String device, String kind, double target) {
        Logger.recordOutput("SelfTest/Active", !name.isEmpty());
        Logger.recordOutput("SelfTest/Step", name);
        Logger.recordOutput("SelfTest/Device", device);
        Logger.recordOutput("SelfTest/Kind", kind);
        Logger.recordOutput("SelfTest/Target", target);
    }
}
