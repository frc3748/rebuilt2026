package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;

public class DynamicPathGenerator {
    private static PathConstraints getConstraints(boolean f) {
        return DriveConstants.pathConstraint;
    }

    private static PathConstraints getConstraints(Optional<PathConstraints> constraint) {
         PathConstraints pathConstraints;

        if (constraint.isPresent()) {
            pathConstraints = constraint.get();
        } else {
            pathConstraints = getConstraints(true);
        }

        return pathConstraints;
    }

    public static Command pathfindAuto(Pose2d desiredPose, Optional<PathConstraints> constraint) {
            return AutoBuilder.pathfindToPose(desiredPose, getConstraints(constraint));
    }

    public static Command pathfindAuto(Pose2d desiredPose) {
        return pathfindAuto(desiredPose, Optional.empty());
    }

    public static Command pathfindAuto(PathPlannerPath pathToFollow, Optional<PathConstraints> constraint) {
        return AutoBuilder.pathfindThenFollowPath(pathToFollow, getConstraints(constraint));
    }

    public static Command pathfindAuto(PathPlannerPath pathToFollow) {
        return pathfindAuto(pathToFollow, Optional.empty());
    }

    public static PathPlannerPath getPathFromWaypoints(List<Waypoint> waypoints, Optional<PathConstraints> constraint, GoalEndState goalEndState) {
        return new PathPlannerPath(waypoints, getConstraints(constraint), null, goalEndState);
    }

    public static PathPlannerPath getPathFromWaypoints(List<Waypoint> waypoints, Optional<PathConstraints> constraint, IdealStartingState startingState, GoalEndState goalEndState) {
        return new PathPlannerPath(waypoints, getConstraints(constraint), startingState, goalEndState);
    }

    public static void warmupInit() {
        PathfindingCommand.warmupCommand().schedule();
    }
}
