package frc.robot.commands.autos;

import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotState;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.AutoCommands.AutoClass;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class Autos {
    private static Command follow(Map<String, PathPlannerPath> paths, String name) {
        return AutoBuilder.followPath(paths.get(name));
    }

    private static Command intake(RobotState state, Intake.State intakeState) {
        return state.getIntake().transitionCommand(intakeState);
    }

    private static Command shooter(RobotState state, Shooter.State shooterState) {
        return state.getShooter().transitionCommand(shooterState);
    }

    private static Command flywheel(RobotState state, Flywheel.State flywheelState) {
        return state.getShooter().getFlywheel().transitionCommand(flywheelState);
    }

    private static Command requestIntake(RobotState state, Intake.State intakeState) {
        return state.getIntake().transitionCommand(intakeState, false);
    }

    private static Command requestShooter(RobotState state, Shooter.State shooterState) {
        return state.getShooter().transitionCommand(shooterState, false);
    }

    private static Command shake(RobotState state, double seconds) {
        return ActionCommands.shakeIntake(state).withTimeout(seconds);
    }

    private static Command aim(RobotState state) {
        return ActionCommands.aimAtHub(state);
    }

    private static Command turn(RobotState state) {
        return ActionCommands.turnToHub(state);
    }

    private static Command nudge(RobotState state, double metersForward) {
        return new DeferredCommand(() -> {
            Pose2d target = state.getLatestFieldToRobot().getValue()
                    .plus(new Transform2d(new Translation2d(metersForward, 0), new Rotation2d()));
            return new AutoAlignToPoseCommand(state.getDrive(), state, target, 1);
        }, Set.of(state.getDrive()));
    }

    private static Command shootFromStart(RobotState state, Map<String, PathPlannerPath> paths, String startPath,
            double shakeSeconds) {
        return Commands.sequence(
                Commands.parallel(
                        follow(paths, startPath),
                        intake(state, Intake.State.IDLE),
                        shooter(state, Shooter.State.HUB_TRACKING)),
                aim(state),
                shooter(state, Shooter.State.SHOOTING),
                shake(state, shakeSeconds),
                shooter(state, Shooter.State.HUB_TRACKING));
    }

    public static class centerOnlyStarting8 extends AutoClass {
        public centerOnlyStarting8() {
            name = "Center Only Starting 8 (GAME)";
            sequentialPathStrings = new String[] { "Start Center to Home Center" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> shootFromStart(state, paths, "Start Center to Home Center", 10));
        }
    }

    public static class centerOnlyStarting8Climb extends AutoClass {
        public centerOnlyStarting8Climb() {
            name = "Center Only Starting 8 Climb (GAME)";
            sequentialPathStrings = new String[] { "Start Center to Home Center" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return afterAuto(state, "Center Only Starting 8 (GAME)", ActionCommands.autoClimb(state));
        }
    }

    public static class depotOnlyStarting8 extends AutoClass {
        public depotOnlyStarting8() {
            name = "Depot Only Starting 8 (GAME)";
            sequentialPathStrings = new String[] { "Start Depot Side To Home Depot" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> shootFromStart(state, paths, "Start Depot Side To Home Depot", 10));
        }
    }

    public static class depotOnlyStarting8Climb extends AutoClass {
        public depotOnlyStarting8Climb() {
            name = "Depot Only Starting 8 Climb (GAME)";
            sequentialPathStrings = new String[] { "Start Depot Side To Home Depot" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return afterAuto(state, "Depot Only Starting 8 (GAME)", ActionCommands.autoClimb(state));
        }
    }

    public static class depotSideToDepot extends AutoClass {
        public depotSideToDepot() {
            name = "Depot Side To Depot (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot Side to Home Depot",
                    "Home Depot to Depot",
                    "Depot Intaking",
                    "Depot Intaking to Depot"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Home Depot"),
                            intake(state, Intake.State.IDLE),
                            shooter(state, Shooter.State.HUB_TRACKING)),
                    follow(paths, "Home Depot to Depot"),
                    Commands.parallel(
                            follow(paths, "Depot Intaking"),
                            intake(state, Intake.State.INTAKE)),
                    follow(paths, "Depot Intaking to Depot"),
                    turn(state).withTimeout(2),
                    Commands.parallel(
                            intake(state, Intake.State.IDLE),
                            shooter(state, Shooter.State.SHOOTING)),
                    shake(state, 14),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class depotSideToDepotClimb extends AutoClass {
        public depotSideToDepotClimb() {
            name = "Depot Side To Depot Climb (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot Side to Home Depot",
                    "Home Depot to Depot",
                    "Depot Intaking"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return afterAuto(state, "Depot Side To Depot (GAME)", ActionCommands.autoClimb(state));
        }
    }

    public static class depotSideToDepotEndAtMid extends AutoClass {
        public depotSideToDepotEndAtMid() {
            name = "Depot Side To Depot End at Mid (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot Side to Depot",
                    "Depot Intaking",
                    "Depot to Mid Under Trench",
                    "Mid Depot Side Half Sweep"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Depot"),
                            intake(state, Intake.State.IDLE)),
                    aim(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 3.5),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Depot Intaking"),
                            intake(state, Intake.State.INTAKE)),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(2),
                    follow(paths, "Depot to Mid Under Trench"),
                    Commands.parallel(
                            follow(paths, "Mid Depot Side Half Sweep"),
                            intake(state, Intake.State.INTAKE),
                            shooter(state, Shooter.State.PASS_TRACKING)),
                    intake(state, Intake.State.IDLE)));
        }
    }

    public static class hpOnlyStarting8 extends AutoClass {
        public hpOnlyStarting8() {
            name = "HP Only Starting 8 (GAME)";
            sequentialPathStrings = new String[] { "Start HP Side To Home HP" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> shootFromStart(state, paths, "Start HP Side To Home HP", 10));
        }
    }

    public static class hpOnlyStarting8Climb extends AutoClass {
        public hpOnlyStarting8Climb() {
            name = "HP Only Starting 8 Climb (GAME)";
            sequentialPathStrings = new String[] { "Start HP Side To Home HP" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return afterAuto(state, "HP Only Starting 8 (GAME)", ActionCommands.autoClimb(state));
        }
    }

    public static class hpSideToHP extends AutoClass {
        public hpSideToHP() {
            name = "HP Side To HP (GAME)";
            sequentialPathStrings = new String[] {
                    "Start HP Side to Home HP",
                    "Home HP to HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shootFromStart(state, paths, "Start HP Side to Home HP", 4),
                    Commands.parallel(
                            follow(paths, "Home HP to HP"),
                            shooter(state, Shooter.State.HUB_TRACKING)),
                    aim(state),
                    shooter(state, Shooter.State.SHOOTING),
                    Commands.waitSeconds(8),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class hpSideToHPClimb extends AutoClass {
        public hpSideToHPClimb() {
            name = "HP Side To HP Climb (GAME)";
            sequentialPathStrings = new String[] {
                    "Start HP Side to Home HP",
                    "Home HP to HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return afterAuto(state, "HP Side To HP (GAME)", ActionCommands.autoClimb(state));
        }
    }

    public static class hpSideToHPEndAtMid extends AutoClass {
        public hpSideToHPEndAtMid() {
            name = "HP Side To HP End at Mid (GAME)";
            sequentialPathStrings = new String[] {
                    "Start HP Side to HP",
                    "HP to Mid",
                    "Mid HP Side Half Sweep"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    Commands.parallel(
                            follow(paths, "Start HP Side to HP"),
                            intake(state, Intake.State.IDLE)),
                    aim(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 4),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    follow(paths, "HP to Mid"),
                    Commands.parallel(
                            follow(paths, "Mid HP Side Half Sweep"),
                            intake(state, Intake.State.INTAKE),
                            shooter(state, Shooter.State.PASS_TRACKING)),
                    intake(state, Intake.State.IDLE)));
        }
    }

    public static class depotSideDepotMidHalfSweep extends AutoClass {
        public depotSideDepotMidHalfSweep() {
            name = "Depot Side Depot Mid Half Sweep (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot Side to Depot",
                    "Depot Intaking",
                    "Depot to Mid Under Trench",
                    "Mid Depot Side Sweep",
                    "Mid HP Side to Home HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Depot"),
                            requestIntake(state, Intake.State.STOW),
                            requestShooter(state, Shooter.State.HUB_TRACKING)),
                    Commands.parallel(
                            follow(paths, "Depot Intaking"),
                            requestIntake(state, Intake.State.INTAKE)),
                    Commands.parallel(
                            requestIntake(state, Intake.State.STOW),
                            Commands.sequence(
                                    requestShooter(state, Shooter.State.SHOOTING),
                                    shake(state, 4),
                                    requestShooter(state, Shooter.State.HUB_TRACKING)),
                            Commands.sequence(
                                    Commands.waitSeconds(1),
                                    follow(paths, "Depot to Mid Under Trench"))),
                    Commands.parallel(
                            follow(paths, "Mid Depot Side Sweep"),
                            requestIntake(state, Intake.State.INTAKE)),
                    Commands.parallel(
                            follow(paths, "Mid HP Side to Home HP"),
                            requestIntake(state, Intake.State.STOW)),
                    Commands.parallel(
                            requestIntake(state, Intake.State.STOW),
                            requestShooter(state, Shooter.State.SHOOTING),
                            shake(state, 5.5)),
                    requestShooter(state, Shooter.State.HUB_TRACKING),
                    ActionCommands.autoClimb(state)));
        }
    }

    public static class depotSideQuickShootClimb extends AutoClass {
        public depotSideQuickShootClimb() {
            name = "Depot Side Quick Shoot Climb (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot Side to Mid Intake",
                    "Mid Intake to Start Depot Side",
                    "Start Depot Side to Mid Intake Second",
                    "Mid Intake to Start Depot Side Second"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shooter(state, Shooter.State.HUB_TRACKING),
                    intake(state, Intake.State.INTAKE),
                    follow(paths, "Start Depot Side to Mid Intake"),
                    Commands.parallel(
                            follow(paths, "Mid Intake to Start Depot Side"),
                            Commands.sequence(Commands.waitSeconds(0.2), intake(state, Intake.State.IDLE))),
                    turn(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 3),
                    intake(state, Intake.State.IDLE),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Mid Intake Second"),
                            Commands.sequence(Commands.waitSeconds(0.6), intake(state, Intake.State.INTAKE))),
                    Commands.parallel(
                            follow(paths, "Mid Intake to Start Depot Side Second"),
                            Commands.sequence(Commands.waitSeconds(0.2), intake(state, Intake.State.IDLE))),
                    turn(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 8),
                    intake(state, Intake.State.IDLE),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class hpSideQuickShootClimb extends AutoClass {
        public hpSideQuickShootClimb() {
            name = "HP Side Quick Shoot Climb (GAME)";
            sequentialPathStrings = new String[] {
                    "Start HP Side to Mid Intake",
                    "Mid Intake to Start HP Side",
                    "Start HP Side to Mid Intake Second"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shooter(state, Shooter.State.HUB_TRACKING),
                    intake(state, Intake.State.INTAKE),
                    Commands.waitSeconds(0.25),
                    follow(paths, "Start HP Side to Mid Intake"),
                    Commands.parallel(
                            follow(paths, "Mid Intake to Start HP Side"),
                            Commands.sequence(Commands.waitSeconds(0.2), intake(state, Intake.State.IDLE))),
                    turn(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 5),
                    intake(state, Intake.State.IDLE),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Start HP Side to Mid Intake Second"),
                            Commands.sequence(Commands.waitSeconds(0.6), intake(state, Intake.State.INTAKE))),
                    nudge(state, 1),
                    Commands.parallel(
                            follow(paths, "Mid Intake to Start HP Side"),
                            Commands.sequence(Commands.waitSeconds(0.2), intake(state, Intake.State.IDLE))),
                    turn(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 8),
                    intake(state, Intake.State.IDLE),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class depotSideCircutShoot extends AutoClass {
        public depotSideCircutShoot() {
            name = "Depot Side Circut Shoot (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot Side to Mid Intake Circut",
                    "Start Depot Side to Mid Intake Circut Second"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shooter(state, Shooter.State.HUB_TRACKING),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(0.25),
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Mid Intake Circut"),
                            Commands.sequence(Commands.waitSeconds(1.4), intake(state, Intake.State.INTAKE)),
                            Commands.sequence(Commands.waitSeconds(3.4), intake(state, Intake.State.IDLE))),
                    aim(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 5),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Mid Intake Circut Second"),
                            Commands.sequence(Commands.waitSeconds(1.8), intake(state, Intake.State.INTAKE)),
                            Commands.sequence(Commands.waitSeconds(4), intake(state, Intake.State.IDLE))),
                    aim(state),
                    shooter(state, Shooter.State.SHOOTING),
                    shake(state, 5),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class depotSideBlair extends AutoClass {
        public depotSideBlair() {
            name = "Depot Side Blair (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot to Intake Blair",
                    "Mid Depot to Intake Depot Side Blair",
                    "Start Depot to Intake Second Blair",
                    "Mid Depot to Intake Depot Side Second Blair"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shooter(state, Shooter.State.HUB_TRACKING),
                    intake(state, Intake.State.IDLE),
                    Commands.parallel(
                            follow(paths, "Start Depot to Intake Blair"),
                            Commands.sequence(Commands.waitSeconds(1.3), intake(state, Intake.State.INTAKE))),
                    intake(state, Intake.State.IDLE),
                    flywheel(state, Flywheel.State.SHOOT),
                    follow(paths, "Mid Depot to Intake Depot Side Blair"),
                    Commands.parallel(
                            turn(state),
                            shooter(state, Shooter.State.SHOOTING)),
                    shake(state, 3),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(1),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Start Depot to Intake Second Blair"),
                            Commands.sequence(Commands.waitSeconds(1.3), intake(state, Intake.State.INTAKE))),
                    intake(state, Intake.State.IDLE),
                    flywheel(state, Flywheel.State.SHOOT),
                    follow(paths, "Mid Depot to Intake Depot Side Second Blair"),
                    Commands.parallel(
                            turn(state),
                            shooter(state, Shooter.State.SHOOTING)),
                    shake(state, 11),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(1),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class hpSideBlair extends AutoClass {
        public hpSideBlair() {
            name = "HP Side Blair (GAME)";
            sequentialPathStrings = new String[] {
                    "Start HP to Intake Blair",
                    "Mid HP to Intake HP Side Blair",
                    "Start HP to Intake Second Blair",
                    "Mid HP to Intake HP Side Second Blair"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shooter(state, Shooter.State.HUB_TRACKING),
                    intake(state, Intake.State.IDLE),
                    Commands.parallel(
                            follow(paths, "Start HP to Intake Blair"),
                            Commands.sequence(Commands.waitSeconds(1.3), intake(state, Intake.State.INTAKE))),
                    intake(state, Intake.State.IDLE),
                    flywheel(state, Flywheel.State.SHOOT),
                    follow(paths, "Mid HP to Intake HP Side Blair"),
                    Commands.parallel(
                            turn(state),
                            shooter(state, Shooter.State.SHOOTING)),
                    shake(state, 3.5),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(0.5),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Start HP to Intake Second Blair"),
                            Commands.sequence(Commands.waitSeconds(1.3), intake(state, Intake.State.INTAKE))),
                    intake(state, Intake.State.IDLE),
                    flywheel(state, Flywheel.State.SHOOT),
                    follow(paths, "Mid HP to Intake HP Side Second Blair"),
                    Commands.parallel(
                            turn(state),
                            shooter(state, Shooter.State.SHOOTING)),
                    shake(state, 11.5),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(0.5),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }

    public static class depotSideBump extends AutoClass {
        public depotSideBump() {
            name = "Depot Side Bump (GAME)";
            sequentialPathStrings = new String[] {
                    "Start Depot to Intake Blair",
                    "Mid Depot to Intake Bump",
                    "Start Depot Side to Intake Bump Second",
                    "Mid Depot to Intake Bump"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> Commands.sequence(
                    shooter(state, Shooter.State.HUB_TRACKING),
                    intake(state, Intake.State.IDLE),
                    Commands.parallel(
                            follow(paths, "Start Depot to Intake Blair"),
                            Commands.sequence(Commands.waitSeconds(1.3), intake(state, Intake.State.INTAKE))),
                    flywheel(state, Flywheel.State.SHOOT),
                    follow(paths, "Mid Depot to Intake Bump"),
                    Commands.parallel(
                            turn(state),
                            Commands.sequence(
                                    intake(state, Intake.State.IDLE),
                                    shooter(state, Shooter.State.SHOOTING))),
                    shake(state, 3),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(1),
                    shooter(state, Shooter.State.HUB_TRACKING),
                    Commands.parallel(
                            follow(paths, "Start Depot Side to Intake Bump Second"),
                            Commands.sequence(Commands.waitSeconds(1.3), intake(state, Intake.State.INTAKE))),
                    flywheel(state, Flywheel.State.SHOOT),
                    follow(paths, "Mid Depot to Intake Bump"),
                    Commands.parallel(
                            turn(state),
                            Commands.sequence(
                                    intake(state, Intake.State.IDLE),
                                    shooter(state, Shooter.State.SHOOTING))),
                    shake(state, 11),
                    intake(state, Intake.State.IDLE),
                    Commands.waitSeconds(1),
                    shooter(state, Shooter.State.HUB_TRACKING)));
        }
    }
}
