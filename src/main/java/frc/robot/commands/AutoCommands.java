package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotState;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.DynamicPathGenerator;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class AutoCommands {
    public static class AutoClass {
        public String[] sequentialPathStrings = new String[0];
        public String name;

        public Command getCommand(RobotState state) {
            return Commands.print("Unfilled");
        }

        public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
            try {
                return new ArrayList<>(getMapPath(sequentialPathStrings).values());
            } catch (Exception e) {
                return new ArrayList<>();
            }
        }

        protected Command build(RobotState state, Function<Map<String, PathPlannerPath>, Command> steps) {
            try {
                Map<String, PathPlannerPath> paths = getMapPath(sequentialPathStrings);
                return Commands.sequence(
                        Commands.runOnce(() -> setRobotPoseToStartingPath(paths.get(sequentialPathStrings[0]), state)),
                        steps.apply(paths))
                        .withName(name);
            } catch (Exception e) {
                return Commands.print("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }

        protected Command afterAuto(RobotState state, String baseName, Command next) {
            return getAutoByName(state, baseName).get().getCommand(state).andThen(next).withName(name);
        }

        protected void setRobotPoseToStartingPath(PathPlannerPath path, RobotState state) {
            if (path.getStartingHolonomicPose().isEmpty()) {
                Elastic.sendNotification(new Notification()
                        .withTitle("Path Error")
                        .withDescription("Unable to set pose")
                        .withLevel(NotificationLevel.ERROR));
                return;
            }
            Pose2d start = path.getStartingHolonomicPose().get();
            state.getDrive().setPose(state.isRedAlliance() ? RobotState.flipPoseForRed(start) : start);
        }
    }

    private static final List<AutoClass> availableAutos = initializeAutos();

    private static List<AutoClass> initializeAutos() {
        List<AutoClass> autos = new ArrayList<>();
        autos.add(new testAuto());
        autos.add(new waypointTestAuto());
        autos.add(new pathfindingTemplate());

        for (Class<?> clazz : Autos.class.getDeclaredClasses()) {
            if (AutoClass.class.isAssignableFrom(clazz)) {
                try {
                    autos.add((AutoClass) clazz.getDeclaredConstructor().newInstance());
                } catch (Exception e) {
                    System.out.println("Skipping " + clazz.getSimpleName() + ": " + e.getMessage());
                }
            }
        }
        return autos;
    }

    public static Optional<AutoClass> getAutoByName(RobotState state, String name) {
        if ("CUSTOM AUTO (GAME)".equals(name)) {
            return Optional.of(state.getCustomAutoBuilder());
        }
        return availableAutos.stream().filter(auto -> auto.name.equals(name)).findFirst();
    }

    public static Map<String, PathPlannerPath> getMapPath(String[] pathNames) throws Exception {
        Map<String, PathPlannerPath> paths = new HashMap<>();
        for (String pathName : pathNames) {
            paths.put(pathName, PathPlannerPath.fromPathFile(pathName));
        }
        return paths;
    }

    public static class testAuto extends AutoClass {
        public testAuto() {
            name = "Apple (GAME)";
            sequentialPathStrings = new String[] { "TESTONE" };
        }

        @Override
        public Command getCommand(RobotState state) {
            return build(state, paths -> AutoBuilder.followPath(paths.get("TESTONE")));
        }
    }

    public static class waypointTestAuto extends AutoClass {
        private final PathPlannerPath path = ActionCommands.waypointTestPath();

        public waypointTestAuto() {
            name = "WAYPOINT (GAME)";
        }

        @Override
        public Command getCommand(RobotState state) {
            return AutoBuilder.followPath(path).withName(name);
        }

        @Override
        public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
            return List.of(path);
        }
    }

    public static class pathfindingTemplate extends AutoClass {
        private final List<Supplier<Command>> pathfindCommands = new ArrayList<>();
        private final List<Pose2d> goalPoses = new ArrayList<>();
        private final PathConstraints constraints = DriveConstants.pathConstraint;

        public pathfindingTemplate() {
            name = "Pathfinding (GAME)";

            Pose2d goalPose = new Pose2d(6, 5, Rotation2d.fromDegrees(180));
            pathfindCommands.add(() -> DynamicPathGenerator.pathfindAuto(goalPose));
            goalPoses.add(goalPose);

            Pose2d depotGoalPose = new Pose2d(
                    VisionConstants.Outpost.centerPoint.getX(),
                    VisionConstants.Outpost.centerPoint.getY(),
                    Rotation2d.fromDegrees(0));
            pathfindCommands.add(() -> DynamicPathGenerator.pathfindAuto(new Pose2d()));
            goalPoses.add(depotGoalPose);
        }

        @Override
        public Command getCommand(RobotState state) {
            return Commands.sequence(
                    Commands.runOnce(() -> state.getDrive().setPose(new Pose2d())),
                    new DeferredCommand(() -> pathfindCommands.get(0).get(), Set.of(state.getDrive())),
                    new DeferredCommand(() -> pathfindCommands.get(1).get(), Set.of(state.getDrive())))
                    .withName(name);
        }

        @Override
        public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
            Pose2d robot = state.getLatestFieldToRobot().getValue();
            return List.of(
                    DynamicPathGenerator.getPathFromWaypoints(
                            PathPlannerPath.waypointsFromPoses(robot, goalPoses.get(0)),
                            Optional.of(constraints),
                            new IdealStartingState(0, robot.getRotation()),
                            new GoalEndState(0, goalPoses.get(0).getRotation())),
                    DynamicPathGenerator.getPathFromWaypoints(
                            PathPlannerPath.waypointsFromPoses(goalPoses.get(0), goalPoses.get(1)),
                            Optional.of(constraints),
                            new IdealStartingState(0, goalPoses.get(0).getRotation()),
                            new GoalEndState(0, goalPoses.get(1).getRotation())));
        }
    }
}
