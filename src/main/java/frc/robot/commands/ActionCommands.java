package frc.robot.commands;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotState;
import frc.robot.commands.AutoAlignToPoseCommand.AlignType;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.DynamicPathGenerator;
import frc.robot.util.GetTuned;
import frc.robot.util.Util;

public class ActionCommands {
    public static PathPlannerPath waypointTestPath() {
        return DynamicPathGenerator.getPathFromWaypoints(
                PathPlannerPath.waypointsFromPoses(
                        new Pose2d(),
                        new Pose2d(1, 2, Rotation2d.fromDegrees(90)),
                        new Pose2d(5, 5, Rotation2d.fromDegrees(0))),
                Optional.empty(),
                new GoalEndState(0, new Rotation2d()));
    }

    public static Command shakeIntake(RobotState state) {
        return Commands.sequence(
                state.getIntake().transitionCommand(Intake.State.SHAKE),
                Commands.waitSeconds(0.6),
                state.getIntake().transitionCommand(Intake.State.IDLE),
                Commands.waitSeconds(0.6))
                .repeatedly();
    }

    public static Command aimAtHub(RobotState state) {
        return aimAtHub(state, AlignType.DEFAULT);
    }

    public static Command turnToHub(RobotState state) {
        return aimAtHub(state, AlignType.ROTATION);
    }

    private static Command aimAtHub(RobotState state, AlignType alignType) {
        return new DeferredCommand(() -> {
            Pose2d pose = state.getLatestFieldToRobot().getValue();
            Pose2d aimed = new Pose2d(pose.getX(), pose.getY(), state.getDrive().getAimRotationForHub());
            return new AutoAlignToPoseCommand(state.getDrive(), state, aimed, 1, alignType);
        }, Set.of(state.getDrive()));
    }

    public static Command aimAndShoot(RobotState state) {
        Shooter shooter = state.getShooter();
        return Commands.sequence(
                Commands.runOnce(() -> shooter.requestTransition(Shooter.State.HUB_TRACKING)),
                Commands.runOnce(() -> shooter.requestTransition(Shooter.State.SHOOTING)));
    }

    public static Command shootOrPassBasedOnPos(RobotState state) {
        Shooter shooter = state.getShooter();
        return new DeferredCommand(
                () -> shooter.transitionCommand(state.shouldShootHub() ? Shooter.State.SHOOTING : Shooter.State.PASSING),
                Set.of(shooter));
    }

    public static Command trackBasedOnPos(RobotState state) {
        Shooter shooter = state.getShooter();
        return new DeferredCommand(
                () -> shooter.transitionCommand(
                        state.shouldShootHub() ? Shooter.State.HUB_TRACKING : Shooter.State.PASS_TRACKING),
                Set.of(shooter));
    }

    public static Command autoClimb(RobotState state) {
        return new DeferredCommand(() -> {
            Translation2d center = VisionConstants.Tower.centerPoint;
            double rotationDeg = 180.0;
            double direction = 1.0;

            if (state.isRedAlliance()) {
                rotationDeg = 0.0;
                direction = -1.0;
                center = Util.flipRedBlueXY(new Translation3d(center)).toTranslation2d();
            }

            Pose2d preClimbPose = new Pose2d(
                    center.plus(new Translation2d(0.1 * direction, 0)), Rotation2d.fromDegrees(rotationDeg));
            Pose2d climbPose = new Pose2d(
                    center.plus(new Translation2d(-0.15 * direction, 0)), Rotation2d.fromDegrees(rotationDeg));

            return Commands.sequence(
                    state.getIntake().transitionCommand(Intake.State.CLIMB_TOW),
                    new AutoAlignToPoseCommand(state.getDrive(), state, preClimbPose, 1),
                    state.getClimb().transitionCommand(Climb.State.UP),
                    Commands.waitSeconds(0.35),
                    new AutoAlignToPoseCommand(state.getDrive(), state, climbPose, 1).withTimeout(1),
                    state.getClimb().transitionCommand(Climb.State.DOWN));
        }, Set.of(state.getDrive(), state.getClimb(), state.getIntake(), state.getShooter()));
    }

    public static Command goToFixedPosAndShoot(RobotState state) {
        return new DeferredCommand(() -> {
            Pose2d goalPose = VisionConstants.Hub.nearFace.transformBy(
                    new Transform2d(new Translation2d(Units.inchesToMeters(80), 0), new Rotation2d()));
            if (state.isRedAlliance()) {
                goalPose = RobotState.flipPoseForRed(goalPose);
            }

            double shooterRps = GetTuned.getNumber("FixedPos/RPS", 9.8);
            double hoodPos = GetTuned.getNumber("FixedPos/Hood", 0);
            double hoodFF = GetTuned.getNumber("FixedPos/Hood FF", 0);
            Flywheel flywheel = state.getShooter().getFlywheel();
            Hood hood = state.getShooter().getHood();

            return Commands.sequence(
                    new AutoAlignToPoseCommand(state.getDrive(), state, goalPose, 1),
                    Commands.runOnce(() -> {
                        flywheel.setOverride(() -> flywheel.spin(shooterRps));
                        hood.setOverride(() -> hood.setPos(hoodPos, hoodFF));
                    }));
        }, Set.of(state.getDrive(), state.getShooter()));
    }
}
