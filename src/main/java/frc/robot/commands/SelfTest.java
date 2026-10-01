package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.motor.Motor;
import frc.robot.util.motor.PosMotor;
import frc.robot.util.state.StateMachine;

public final class SelfTest {
    private static final double kDriveVolts = 1.5;
    private static final double kDriveSeconds = 1.0;
    private static final double kTurnSeconds = 1.0;
    private static final double kSpinVolts = 2.0;
    private static final double kSpinSeconds = 0.8;
    private static final double kMoveSeconds = 1.5;
    private static final double kRestSeconds = 0.4;

    private SelfTest() {}

    public static Command build(RobotState state) {
        Drive drive = state.getDrive();
        List<StateMachine<?>> machines = new ArrayList<>();
        collectMechanisms(state, machines);

        List<Command> steps = new ArrayList<>();
        steps.add(step("Drive forward", "Drive", "drive", Double.NaN,
                Commands.run(() -> drive.runCharacterization(kDriveVolts), drive).withTimeout(kDriveSeconds)
                        .finallyDo(drive::stop)));
        steps.add(step("Turn to 90", "Drive", "turn", 90.0,
                Commands.run(() -> drive.runModuleAngles(Rotation2d.kCCW_90deg), drive).withTimeout(kTurnSeconds)));
        steps.add(step("Turn to 0", "Drive", "turn", 0.0,
                Commands.run(() -> drive.runModuleAngles(Rotation2d.kZero), drive).withTimeout(kTurnSeconds)
                        .finallyDo(drive::stop)));
        for (StateMachine<?> machine : machines) {
            for (Motor motor : machine.getMotors()) {
                steps.add(motorStep(machine, motor));
            }
        }

        return Commands.sequence(
                        Commands.runOnce(() -> machines.forEach(machine -> machine.setOverride(holdStill(machine, null)))),
                        Commands.sequence(steps.toArray(Command[]::new)))
                .finallyDo(() -> {
                    machines.forEach(StateMachine::clearOverride);
                    record("", "", "", Double.NaN);
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
                    Commands.runOnce(() -> machine.setOverride(
                            with(holdStill(machine, motor), () -> motor.holdPosition(start[0])))),
                    Commands.waitSeconds(kMoveSeconds)));
        }
        if (motor instanceof PosMotor) {
            return step(motor.getName(), motor.getName(), "hold", Double.NaN, Commands.waitSeconds(kRestSeconds));
        }
        return step(motor.getName(), motor.getName(), "spin", kSpinVolts, Commands.sequence(
                Commands.runOnce(() -> machine.setOverride(
                        with(holdStill(machine, motor), () -> motor.setVoltage(kSpinVolts)))),
                Commands.waitSeconds(kSpinSeconds),
                Commands.runOnce(() -> machine.setOverride(holdStill(machine, null))),
                Commands.waitSeconds(kRestSeconds)));
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
