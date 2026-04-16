package frc.robot.commands.autos;

import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoAlignToPoseCommand.AlignType;
import frc.robot.commands.AutoCommands.AutoClass;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.State;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.vision.VisionConstants;

public class Autos {

    // Aarush's Autos

    // Simple Autos
    // Center Start
    public static class centerOnlyStarting8 extends AutoClass {
        public centerOnlyStarting8() {
            this.name = "Center Only Starting 8 (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Center to Home Center"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Center to Home Center")),
                                state.getIntake().transitionCommand(Intake.State.IDLE),
                                state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),

                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        // new WaitCommand(10),
                        ActionCommands.shakeIntake(state).withTimeout(10),

                        // new WaitCommand(10),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerOnlyStarting8Climb extends AutoClass {
        public centerOnlyStarting8Climb() {
            this.name = "Center Only Starting 8 Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Center to Home Center",
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return AutoCommands.getAutoByName(state, "Center Only Starting 8 (GAME)").get().getCommand(state)
                    .andThen(ActionCommands.autoClimb(state)).withName(this.name);
        }
    }

    // Depot Start
    public static class depotOnlyStarting8 extends AutoClass {
        public depotOnlyStarting8() {
            this.name = "Depot Only Starting 8 (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side To Home Depot"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side To Home Depot")),
                                state.getIntake().transitionCommand(Intake.State.IDLE),
                                state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),

                        ActionCommands.shakeIntake(state).withTimeout(10),
                        // new WaitCommand(10),
                        // new WaitCommand(10),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class depotOnlyStarting8Climb extends AutoClass {
        public depotOnlyStarting8Climb() {
            this.name = "Depot Only Starting 8 Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side To Home Depot"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return AutoCommands.getAutoByName(state, "Depot Only Starting 8 (GAME)").get().getCommand(state)
                    .andThen(ActionCommands.autoClimb(state)).withName(this.name);
        }
    }

    public static class depotSideToDepot extends AutoClass {
        public depotSideToDepot() {
            this.name = "Depot Side To Depot (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side to Home Depot",
                    "Home Depot to Depot",
                    "Depot Intaking",
                    "Depot Intaking to Depot"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side to Home Depot")),
                                state.getIntake().transitionCommand(Intake.State.IDLE),
                                state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)),

                        // new DeferredCommand(() -> {
                        //     Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                        //     return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                        //             currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        // }, Set.of(state.getDrive())),
                        // state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        // ActionCommands.shakeIntake(state).withTimeout(4),
                        // new WaitCommand(4),
                        // state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                        // new WaitCommand(4),

                        AutoBuilder.followPath(pathMap.get("Home Depot to Depot")),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot Intaking")),
                                state.getIntake().transitionCommand(Intake.State.INTAKE)),

                        AutoBuilder.followPath(pathMap.get("Depot Intaking to Depot")),
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())).withTimeout(2),
                        new ParallelCommandGroup(
                                state.getIntake().transitionCommand(Intake.State.IDLE),
                                state.getShooter().transitionCommand(Shooter.State.SHOOTING)),

                        ActionCommands.shakeIntake(state).withTimeout(14),
                        // new WaitCommand(6),
                        // new WaitCommand(6),



                        
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class depotSideToDepotClimb extends AutoClass {
        public depotSideToDepotClimb() {
            this.name = "Depot Side To Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side to Home Depot",
                    "Home Depot to Depot",
                    "Depot Intaking"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return AutoCommands.getAutoByName(state, "Depot Side To Depot (GAME)").get().getCommand(state)
                    .andThen(ActionCommands.autoClimb(state)).withName(this.name);
        }
    }

    // Might change to go to home then depot with no shooting on the move
    public static class depotSideToDepotEndAtMid extends AutoClass {
        public depotSideToDepotEndAtMid() {
            this.name = "Depot Side To Depot End at Mid (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side to Depot",
                    "Depot Intaking",
                    "Depot to Mid Under Trench",
                    "Mid Depot Side Half Sweep"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side to Depot")),
                                state.getIntake().transitionCommand(Intake.State.IDLE)),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(3.5),
                        // new WaitCommand(3.5),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot Intaking")),
                                state.getIntake().transitionCommand(Intake.State.INTAKE)),

                        state.getIntake().transitionCommand(Intake.State.IDLE),

                        new WaitCommand(2),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot to Mid Under Trench"))),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid Depot Side Half Sweep")),
                                state.getIntake().transitionCommand(Intake.State.INTAKE),
                                state.getShooter().transitionCommand(Shooter.State.PASS_TRACKING)),

                        state.getIntake().transitionCommand(Intake.State.IDLE)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    // HP Start
    public static class hpOnlyStarting8 extends AutoClass {
        public hpOnlyStarting8() {
            this.name = "HP Only Starting 8 (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP Side To Home HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start HP Side To Home HP")),
                                state.getIntake().transitionCommand(Intake.State.IDLE),
                                state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(10),
                        // new WaitCommand(10),
                        // new WaitCommand(10),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class hpOnlyStarting8Climb extends AutoClass {
        public hpOnlyStarting8Climb() {
            this.name = "HP Only Starting 8 Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP Side To Home HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return AutoCommands.getAutoByName(state, "HP Only Starting 8 (GAME)").get().getCommand(state)
                    .andThen(ActionCommands.autoClimb(state)).withName(this.name);
        }
    }

    public static class hpSideToHP extends AutoClass {
        public hpSideToHP() {
            this.name = "HP Side To HP (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP Side to Home HP",
                    "Home HP to HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start HP Side to Home HP")),
                                state.getIntake().transitionCommand(Intake.State.IDLE),
                                state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(4),
                        // new WaitCommand(4),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                        // new WaitCommand(4),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Home HP to HP")),
                                state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                        ),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        new WaitCommand(8),
                        // new WaitCommand(8),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class hpSideToHPClimb extends AutoClass {
        public hpSideToHPClimb() {
            this.name = "HP Side To HP Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP Side to Home HP",
                    "Home HP to HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            return AutoCommands.getAutoByName(state, "HP Side To HP (GAME)").get().getCommand(state)
                    .andThen(ActionCommands.autoClimb(state)).withName(this.name);
        }
    }

    // Might change to go to home then HP with no shooting on the move
    public static class hpSideToHPEndAtMid extends AutoClass {
        public hpSideToHPEndAtMid() {
            this.name = "HP Side To HP End at Mid (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP Side to HP",
                    "HP to Mid",
                    "Mid HP Side Half Sweep"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start HP Side to HP")),
                                state.getIntake().transitionCommand(Intake.State.IDLE)),
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(4),
                        // new WaitCommand(4),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP to Mid"))),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid HP Side Half Sweep")),
                                state.getIntake().transitionCommand(Intake.State.INTAKE),
                                state.getShooter().transitionCommand(Shooter.State.PASS_TRACKING)),

                        state.getIntake().transitionCommand(Intake.State.IDLE)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    // ___________________________________________________________________________\\

    public static class depotSideDepotMidHalfSweep extends AutoClass {
        public depotSideDepotMidHalfSweep() {
            this.name = "Depot Side Depot Mid Half Sweep (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side to Depot",
                    "Depot Intaking",
                    "Depot to Mid Under Trench",
                    "Mid Depot Side Sweep",
                    "Mid HP Side to Home HP"
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side to Depot")),
                                new InstantCommand(() -> {
                                    state.getIntake().requestTransition(Intake.State.STOW);
                                }),
                                new InstantCommand(() -> {
                                    state.getShooter().requestTransition(Shooter.State.HUB_TRACKING);
                                })),
                        // MAYBE SHOOT DEPENDING ON HOW LONG IT TAKES TO SPIN UP IF TOO LONG SHOOT AFTER
                        // INTAKING

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot Intaking")),
                                new InstantCommand(() -> {
                                    state.getIntake().requestTransition(Intake.State.INTAKE);
                                })),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> {
                                    state.getIntake().requestTransition(Intake.State.STOW);
                                }),
                                // need to test and figure out timings
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING);
                                        }),
                                        ActionCommands.shakeIntake(state).withTimeout(4),
                                        // new WaitCommand(4),
                                        // new WaitCommand(2), // need to figure out
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.HUB_TRACKING);
                                        })),
                                new SequentialCommandGroup(
                                        new WaitCommand(1), // need to figure out
                                        AutoBuilder.followPath(pathMap.get("Depot to Mid Under Trench")))),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid Depot Side Sweep")),
                                new InstantCommand(() -> {
                                    state.getIntake().requestTransition(Intake.State.INTAKE);
                                })),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid HP Side to Home HP")),
                                new InstantCommand(() -> {
                                    state.getIntake().requestTransition(Intake.State.STOW);
                                })),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> {
                                    state.getIntake().requestTransition(Intake.State.STOW);
                                }),
                                new InstantCommand(() -> {
                                    state.getShooter().requestTransition(Shooter.State.SHOOTING);
                                }),
                                ActionCommands.shakeIntake(state).withTimeout(5.5)
                        // new WaitCommand(5.5)
                        // new WaitCommand(5.5)//need to figure out
                        ),

                        new InstantCommand(() -> {
                            state.getShooter().requestTransition(Shooter.State.HUB_TRACKING);
                        }),
                        ActionCommands.autoClimb(state)

                ).withName(name);

            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class depotSideQuickShootClimb extends AutoClass {
        public depotSideQuickShootClimb() {
            this.name = "Depot Side Quick Shoot Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side to Mid Intake",
                    "Mid Intake to Start Depot Side",
                    "Start Depot Side to Mid Intake Second",
                    "Mid Intake to Start Depot Side Second",
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]),
                                state)),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                        state.getIntake().transitionCommand(Intake.State.INTAKE),
                        // new WaitCommand(0.25),
                        AutoBuilder.followPath(pathMap.get("Start Depot Side to Mid Intake")),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid Intake to Start Depot Side")),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.2),
                                        state.getIntake().transitionCommand(Intake.State.IDLE))),

                        new SequentialCommandGroup(
                                        new DeferredCommand(() -> {
                                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                                            return new AutoAlignToPoseCommand(state.getDrive(), state,
                                                    new Pose2d(currentPose.getX(), currentPose.getY(),
                                                            state.getDrive().getAimRotationForHub()),
                                                    1, AlignType.ROTATION);
                                        }, Set.of(state.getDrive()))),
                                state.getShooter().transitionCommand(Shooter.State.SHOOTING),

                        ActionCommands.shakeIntake(state).withTimeout(3),
                        state.getIntake().transitionCommand(Intake.State.IDLE),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side to Mid Intake Second")),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.6),
                                        state.getIntake().transitionCommand(Intake.State.INTAKE))),


                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid Intake to Start Depot Side Second")),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.2),
                                        state.getIntake().transitionCommand(Intake.State.IDLE))),

                        new SequentialCommandGroup(
                                        new DeferredCommand(() -> {
                                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                                            return new AutoAlignToPoseCommand(state.getDrive(), state,
                                                    new Pose2d(currentPose.getX(), currentPose.getY(),
                                                            state.getDrive().getAimRotationForHub()),
                                                    1, AlignType.ROTATION);
                                        }, Set.of(state.getDrive()))),
                                state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(8),
                        state.getIntake().transitionCommand(Intake.State.IDLE),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                // AutoBuilder.followPath(pathMap.get("Home Depot Far to Ladder Depot"))
                // ActionCommands.autoClimb(state)
                ).withName(this.name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " +
                        e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class hpSideQuickShootClimb extends AutoClass {
        public hpSideQuickShootClimb() {
            this.name = "HP Side Quick Shoot Climb (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP Side to Mid Intake",
                    "Mid Intake to Start HP Side"
                    // "Start HP Side to Mid Intake Second",
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]),
                                state)),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                        state.getIntake().transitionCommand(Intake.State.INTAKE),
                        new WaitCommand(0.25),
                        AutoBuilder.followPath(pathMap.get("Start HP Side to Mid Intake")),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid Intake to Start HP Side")),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.2),
                                        state.getIntake().transitionCommand(Intake.State.IDLE))),

                        new SequentialCommandGroup(
                                        new DeferredCommand(() -> {
                                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                                            return new AutoAlignToPoseCommand(state.getDrive(), state,
                                                    new Pose2d(currentPose.getX(), currentPose.getY(),
                                                            state.getDrive().getAimRotationForHub()),
                                                    1, AlignType.ROTATION);
                                        }, Set.of(state.getDrive()))),
                                state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(5),
                        state.getIntake().transitionCommand(Intake.State.IDLE),
                        // new WaitCommand(5),
                        // new WaitCommand(4),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start HP Side to Mid Intake Second")),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.6),
                                        state.getIntake().transitionCommand(Intake.State.INTAKE))),

                        // allows us to go more inward to collect fuel
                        new DeferredCommand(() -> {
                            // boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) ==
                            // Alliance.Blue;

                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            Translation2d extraTranslation = new Translation2d(1, 0); // from blue side, going more to
                                                                                      // right
                            // if (!isBlue) {
                            // extraTranslation = extraTranslation.times(-1);
                            // }

                            return new AutoAlignToPoseCommand(state.getDrive(), state,
                                    currentPose.plus(new Transform2d(extraTranslation, new Rotation2d())), 1);
                        }, Set.of(state.getDrive())),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Mid Intake to Start HP Side")),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.2),
                                        state.getIntake().transitionCommand(Intake.State.IDLE))),
                        new SequentialCommandGroup(
                                        new DeferredCommand(() -> {
                                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                                            return new AutoAlignToPoseCommand(state.getDrive(), state,
                                                    new Pose2d(currentPose.getX(), currentPose.getY(),
                                                            state.getDrive().getAimRotationForHub()),
                                                    1, AlignType.ROTATION);
                                        }, Set.of(state.getDrive()))),
                                state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(8),
                        state.getIntake().transitionCommand(Intake.State.IDLE),
                        // // new WaitCommand(8),

                        // // new WaitCommand(3),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)

                // AutoBuilder.followPath(pathMap.get("Home HP Far to Ladder HP"))
                // ActionCommands.autoClimb(state)
                ).withName(this.name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " +
                        e.getMessage()).withName(name + " (FAILED)");
            }
        }

    }

    public static class depotSideCircutShoot extends AutoClass {
        public depotSideCircutShoot() {
            this.name = "Depot Side Circut Shoot (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot Side to Mid Intake Circut",
                    "Start Depot Side to Mid Intake Circut Second",
            };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]),
                                state)),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                        state.getIntake().transitionCommand(Intake.State.IDLE),
                        new WaitCommand(0.25),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side to Mid Intake Circut")),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.4),
                                        state.getIntake().transitionCommand(Intake.State.INTAKE)),
                                new SequentialCommandGroup(
                                        new WaitCommand(3.4),
                                        state.getIntake().transitionCommand(Intake.State.IDLE))),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(5),
                        // new WaitCommand(5),
                        // new WaitCommand(4),
                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Start Depot Side to Mid Intake Circut Second")),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.8),
                                        state.getIntake().transitionCommand(Intake.State.INTAKE)),
                                new SequentialCommandGroup(
                                        new WaitCommand(4),
                                        state.getIntake().transitionCommand(Intake.State.IDLE))),

                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(),
                                    currentPose.getY(), state.getDrive().getAimRotationForHub()), 1);
                        }, Set.of(state.getDrive())),
                        state.getShooter().transitionCommand(Shooter.State.SHOOTING),
                        ActionCommands.shakeIntake(state).withTimeout(5),
                        // new WaitCommand(5),

                        state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)).withName(this.name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " +
                        e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    // Other Autos

    public static class depotSideBlair extends AutoClass {
        public depotSideBlair() {
            this.name = "Depot Side Blair (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot to Intake Blair",
                    "Mid Depot to Intake Depot Side Blair",
                    "Start Depot to Intake Second Blair",
                    "Mid Depot to Intake Depot Side Second Blair"
            };
        }

        public Command getCommand(RobotState state) {
            try {
                    Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]),
                                    state)),
                            state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                            state.getIntake().transitionCommand(Intake.State.IDLE),
                            // new WaitCommand(0.25),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Start Depot to Intake Blair")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.3),
                            state.getIntake().transitionCommand(Intake.State.INTAKE)
                        )
                    ),

                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    state.getShooter().getFlywheel().transitionCommand(Flywheel.State.SHOOT),
                    AutoBuilder.followPath(pathMap.get("Mid Depot to Intake Depot Side Blair")),

                    new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(), currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())),
                        new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(true);
                        }),
                        new SequentialCommandGroup(
                            // new WaitCommand(0.2),
                            state.getShooter().transitionCommand(Shooter.State.SHOOTING)
                        )
                    ),

                    new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(false);
                        }),
                    ActionCommands.shakeIntake(state).withTimeout(3),
                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    new WaitCommand(1),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Start Depot to Intake Second Blair")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.3),
                            state.getIntake().transitionCommand(Intake.State.INTAKE)
                        )
                    ),

                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    state.getShooter().getFlywheel().transitionCommand(Flywheel.State.SHOOT),
                    AutoBuilder.followPath(pathMap.get("Mid Depot to Intake Depot Side Second Blair")),
                    
                    new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(), currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())),
                        new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(true);
                        }),
                        new SequentialCommandGroup(
                            // new WaitCommand(0.2),
                            state.getShooter().transitionCommand(Shooter.State.SHOOTING)
                        )
                    ),

                    ActionCommands.shakeIntake(state).withTimeout(11),
                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    new WaitCommand(1),
                    new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(false);
                        }),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)
                ).withName(this.name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " +
                        e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class hpSideBlair extends AutoClass {
        public hpSideBlair() {
            this.name = "HP Side Blair (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start HP to Intake Blair",
                    "Mid HP to Intake HP Side Blair",
                    "Start HP to Intake Second Blair",
                    "Mid HP to Intake HP Side Second Blair"
            };
        }

        public Command getCommand(RobotState state) {
            try {
                    Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]),
                                    state)),
                            state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                            state.getIntake().transitionCommand(Intake.State.IDLE),
                            // new WaitCommand(0.25),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Start HP to Intake Blair")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.3),
                            state.getIntake().transitionCommand(Intake.State.INTAKE)
                        )
                    ),

                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    state.getShooter().getFlywheel().transitionCommand(Flywheel.State.SHOOT),
                    AutoBuilder.followPath(pathMap.get("Mid HP to Intake HP Side Blair")),

                    new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(), currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())),
                        new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(true);
                        }),
                        new SequentialCommandGroup(
                            // new WaitCommand(0.2),
                            state.getShooter().transitionCommand(Shooter.State.SHOOTING)
                        )
                    ),

                    new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(false);
                        }),
                    ActionCommands.shakeIntake(state).withTimeout(3.5),
                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    new WaitCommand(0.5),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Start HP to Intake Second Blair")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.3),
                            state.getIntake().transitionCommand(Intake.State.INTAKE)
                        )
                    ),

                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    state.getShooter().getFlywheel().transitionCommand(Flywheel.State.SHOOT),
                    AutoBuilder.followPath(pathMap.get("Mid HP to Intake HP Side Second Blair")),

                    new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(), currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())),
                        new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(true);
                        }),
                        new SequentialCommandGroup(
                            // new WaitCommand(0.2),
                            state.getShooter().transitionCommand(Shooter.State.SHOOTING)
                        )
                    ),

                    ActionCommands.shakeIntake(state).withTimeout(11.5),
                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(false);
                        }),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)
                ).withName(this.name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " +
                        e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class depotSideBump extends AutoClass {
        public depotSideBump() {
            this.name = "Depot Side Bump (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Start Depot to Intake Blair",
                    "Mid Depot to Intake Bump",
                    "Start Depot Side to Intake Bump Second",
                    "Mid Depot to Intake Bump"
            };
        }

        public Command getCommand(RobotState state) {
            try {
                    Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]),
                                    state)),
                            state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
                            state.getIntake().transitionCommand(Intake.State.IDLE),
                            // new WaitCommand(0.25),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Start Depot to Intake Blair")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.3),
                            state.getIntake().transitionCommand(Intake.State.INTAKE)
                        )
                    ),

                    state.getShooter().getFlywheel().transitionCommand(Flywheel.State.SHOOT),
                    AutoBuilder.followPath(pathMap.get("Mid Depot to Intake Bump")),

                    new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(), currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())),
                        new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(true);
                        }),
                        new SequentialCommandGroup(
                            // new WaitCommand(0.2),
                            state.getIntake().transitionCommand(Intake.State.IDLE),
                            state.getShooter().transitionCommand(Shooter.State.SHOOTING)
                        )
                    ),

                    new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(false);
                        }),
                    ActionCommands.shakeIntake(state).withTimeout(3),
                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    new WaitCommand(1),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Start Depot Side to Intake Bump Second")),
                        new SequentialCommandGroup(
                            new WaitCommand(1.3),
                            state.getIntake().transitionCommand(Intake.State.INTAKE)
                        )
                    ),

                    state.getShooter().getFlywheel().transitionCommand(Flywheel.State.SHOOT),
                    AutoBuilder.followPath(pathMap.get("Mid Depot to Intake Bump")),
                    
                    new ParallelCommandGroup(
                        new DeferredCommand(() -> {
                            Pose2d currentPose = state.getLatestFieldToRobot().getValue();
                            return new AutoAlignToPoseCommand(state.getDrive(), state, new Pose2d(currentPose.getX(), currentPose.getY(), state.getDrive().getAimRotationForHub()), 1, AlignType.ROTATION);
                        }, Set.of(state.getDrive())),
                        new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(true);
                        }),
                        new SequentialCommandGroup(
                            // new WaitCommand(0.2),
                            state.getIntake().transitionCommand(Intake.State.IDLE),
                            state.getShooter().transitionCommand(Shooter.State.SHOOTING)
                        )
                    ),

                    ActionCommands.shakeIntake(state).withTimeout(11),
                    state.getIntake().transitionCommand(Intake.State.IDLE),
                    new WaitCommand(1),
                    new InstantCommand(() -> {
                            state.getShooter().getHood().setAutoOverride(false);
                        }),
                    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING)
                ).withName(this.name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " +
                        e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }
    //Other Autos
    public static class leftDepotClimb extends AutoClass {
        public leftDepotClimb() {
            this.name = "Left Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Center", "Center to Depot", "Depot to Center",
                    "Center to Left Climb" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Left to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.leftToCenter),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Depot")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToDepot),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.depotToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Left Climb")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToClimb),
                                        new InstantCommand(() -> {
                                            // state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when
                                            // climb is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class rightFuelClimb extends AutoClass {
        public rightFuelClimb() {
            this.name = "Right Fuel Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Right Fuel",
                    "Right Fuel to Center", "Center to Right Climb" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.rightToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready//state.getShooter().requestTransition(State.SHOOTING);
                                                                                                          // //
                                                                                                          // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToRFuel),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.rFuelToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Climb")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToClimb),
                                        new InstantCommand(() -> {
                                            // state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when
                                            // climb is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerHPClimb extends AutoClass {
        public centerHPClimb() {
            this.name = "Center HP Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to HP", "HP Pickup", "HP to Center",
                    "Center to Right Climb" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        AutoBuilder.followPath(pathMap.get("Center to HP")),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP Pickup")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToClimb),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP to Center")),
                                new SequentialCommandGroup(
                                        // new WaitCommand(AutosConstants.hpToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Climb")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToClimb),
                                        new InstantCommand(() -> {
                                            // state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when
                                            // climb is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerRightHPClimb extends AutoClass {
        public centerRightHPClimb() {
            this.name = "Center Right HP Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to Right Fuel", "Right Fuel to Center", "Center to HP",
                    "HP Pickup", "HP to Climb" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        // new ParallelCommandGroup(
                        // state.getShooter().requestTransition(Shooter.State.SHOOTING), // uncomment
                        // when shooter is ready
                        // new WaitCommand(AutosConstants.shootingPause)
                        // ),
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToRFuel),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.rFuelToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to HP")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToHP),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        AutoBuilder.followPath(pathMap.get("HP Pickup")),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP to Climb")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.hpToClimb),
                                        new InstantCommand(() -> {
                                            // state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when
                                            // climb is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerLeftDepotClimb extends AutoClass {
        public centerLeftDepotClimb() {
            this.name = "Center Left Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Center to Left Fuel", "Left Fuel to Center", "Center to Depot",
                    "Depot to Climb" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        // new ParallelCommandGroup(
                        // //state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                        // shooter is ready
                        // new WaitCommand(AutosConstants.shootingPause)
                        // ),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Left Fuel")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToLFuel),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Left Fuel to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.lFuelToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Depot")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToDepot),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot to Climb")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.depotToClimb),
                                        new InstantCommand(() -> {
                                            // state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when
                                            // climb is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class leftDepotFuel extends AutoClass {
        public leftDepotFuel() {
            this.name = "Left Depot Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Center", "Center to Depot", "Depot to Center",
                    "Center to Left Fuel", "Left Fuel to Center" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Left to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.leftToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready

                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Depot")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToDepot),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Depot to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.depotToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Left Fuel")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToLFuel),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Left Fuel to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.lFuelToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(Shooter.State.SHOOTING); // uncomment
                                                                                                          // when
                                                                                                          // shooter is
                                                                                                          // ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class centerHPFuel extends AutoClass {
        public centerHPFuel() {
            this.name = "Center HP Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Center to HP", "HP Pickup", "HP to Center",
                    "Center to Right Fuel", "Right Fuel to Center" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to HP")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToHP),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready

                                        }))),
                        AutoBuilder.followPath(pathMap.get("HP Pickup")),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.hpToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                                                                                  // shooter is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToRFuel),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.rFuelToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                                                                                  // shooter is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class rightHPFuel extends AutoClass {
        public rightHPFuel() {
            this.name = "Right HP Fuel (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Center", "Center to Right Fuel",
                    "Right Fuel to Center", "Center to HP", "HP Pickup", "HP to Center" };
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.rightToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                                                                                  // shooter is ready

                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to Right Fuel")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToRFuel),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Right Fuel to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.rightToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                                                                                  // shooter is ready
                                        }))),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Center to HP")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.centerToHP),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when
                                                                                                      // intake is ready
                                        }))),
                        AutoBuilder.followPath(pathMap.get("HP Pickup")),
                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("HP to Center")),
                                new SequentialCommandGroup(
                                        new WaitCommand(AutosConstants.hpToCenter),
                                        new InstantCommand(() -> {
                                            state.getShooter().requestTransition(State.SHOOTING); // uncomment when
                                                                                                  // shooter is ready
                                        }))))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class outpostAuto extends AutoClass {
        Pose2d tagPos;

        public outpostAuto() {
            this.name = "Outpost (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Starting to Outpost - AR",
                    "Outpost to 1st Shooting - AR",
                    "1st Shooting to Depot - AR",
                    "Depot to 2nd Shooting - AR"
            };

            tagPos = VisionConstants.kAprilTagLayout.getTagPose(7).get().toPose2d()
                    .plus(new Transform2d(Units.inchesToMeters(36), Units.inchesToMeters(0),
                            new Rotation2d(Units.degreesToRadians(270))));
        }

        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                        AutoBuilder.followPath(pathMap.get("Starting to Outpost - AR")),
                        // new WaitCommand(2), // Temp seconds amount
                        AutoBuilder.followPath(pathMap.get("Outpost to 1st Shooting - AR")),
                        // ActionCommands.aimAndShoot(state),
                        AutoBuilder.followPath(pathMap.get("1st Shooting to Depot - AR")),
                        // ActionCommands.aimAndShoot(state),
                        AutoBuilder.followPath(pathMap.get("Depot to 2nd Shooting - AR"))
                // new AutoAlignToPoseCommand(state.getDrive(), state, tagPos, 0)
                // ActionCommands.climbUp(state)
                )
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }

    public static class depotAuto extends AutoClass {
        Pose2d tagPos;

        public depotAuto() {
            this.name = "Depot (GAME)";
            this.sequentialPathStrings = new String[] {
                    "Starting to Depot - AR",
                    "Depot to 1st Shooting - AR",
                    "1st Shooting to 2nd Shooting - AR"
            };

            tagPos = VisionConstants.kAprilTagLayout.getTagPose(7).get().toPose2d()
                    .plus(new Transform2d(Units.inchesToMeters(36), Units.inchesToMeters(0),
                            new Rotation2d(Units.degreesToRadians(270))));
        }

        @Override
        public Command getCommand(RobotState state) {
            // state.getDrive().setPose(new Pose2d(0, 10, Rotation2d.kZero));
            // return ActionCommands.autoClimb(state);
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                new ActionCommands();

                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),

                        new ParallelCommandGroup(
                                AutoBuilder.followPath(pathMap.get("Starting to Depot - AR")),
                                new SequentialCommandGroup(
                                        new WaitCommand(1),
                                        new InstantCommand(() -> {
                                            state.getIntake().requestTransition(Intake.State.INTAKE);
                                        }),

                                        new WaitCommand(1.5),
                                        ActionCommands.aimAndShoot(state))),
                        AutoBuilder.followPath(pathMap.get("Depot to 1st Shooting - AR")),
                        new WaitCommand(1),
                        new InstantCommand(() -> {
                            state.getShooter().requestTransition(State.IDLE);
                        }),
                        AutoBuilder.followPath(pathMap.get("1st Shooting to 2nd Shooting - AR")),
                        new InstantCommand(() -> {
                            state.getShooter().requestTransition(State.SHOOTING);
                            state.getIntake().requestTransition(Intake.State.STOW);
                        }),
                        new WaitCommand(5),
                        ActionCommands.autoClimb(state))
                        .withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command").withName(name + " (FAILED)");
            }
        }
    }

}