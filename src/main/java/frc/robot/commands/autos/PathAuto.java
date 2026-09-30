package frc.robot.commands.autos;

import java.util.Map;
import java.util.Optional;
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
import frc.robot.Superstructure;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.game.AllianceFlip;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public abstract class PathAuto extends AutoRoutine {
    protected final RobotState state;
    private final Superstructure robot;
    private Map<String, PathPlannerPath> paths = Map.of();

    protected PathAuto(RobotState state, String name, String... pathNames) {
        super(name, pathNames);
        this.state = state;
        robot = state.getSuperstructure();
    }

    protected abstract Command routine();

    @Override
    public final Command build() {
        try {
            paths = loadPaths();
            PathPlannerPath start = paths.get(firstPathName());
            return Commands.sequence(Commands.runOnce(() -> resetPose(start)), routine()).withName(name());
        } catch (Exception e) {
            return Commands.print("Failed to generate command: " + e.getMessage()).withName(name() + " (FAILED)");
        }
    }

    private void resetPose(PathPlannerPath path) {
        Optional<Pose2d> start = path.getStartingHolonomicPose();
        if (start.isEmpty()) {
            Elastic.sendNotification(new Notification()
                    .withTitle("Path Error")
                    .withDescription("Unable to set pose")
                    .withLevel(NotificationLevel.ERROR));
            return;
        }
        state.getDrive().setPose(AllianceFlip.forAlliance(start.get()));
    }

    protected Command follow(String pathName) {
        return AutoBuilder.followPath(paths.get(pathName));
    }

    protected Command intake(Intake.State intakeState) {
        return robot.intakeCommand(intake -> intake.transitionCommand(intakeState));
    }

    protected Command shooter(Shooter.State shooterState) {
        return robot.shooterCommand(shooter -> shooter.transitionCommand(shooterState));
    }

    protected Command spinUp() {
        return robot.shooterCommand(Shooter::spinUp);
    }

    protected Command requestIntake(Intake.State intakeState) {
        return robot.intakeCommand(intake -> intake.transitionCommand(intakeState, false));
    }

    protected Command requestShooter(Shooter.State shooterState) {
        return robot.shooterCommand(shooter -> shooter.transitionCommand(shooterState, false));
    }

    protected Command shake(double seconds) {
        return Commands.waitSeconds(seconds).deadlineFor(ActionCommands.shakeIntake(state));
    }

    protected Command aim() {
        return ActionCommands.aimAtHub(state);
    }

    protected Command turn() {
        return ActionCommands.turnToHub(state);
    }

    protected Command nudge(double metersForward) {
        return new DeferredCommand(() -> {
            Pose2d target = state.getLatestFieldToRobot().getValue()
                    .plus(new Transform2d(new Translation2d(metersForward, 0), new Rotation2d()));
            return new AutoAlignToPoseCommand(state.getDrive(), state, target, 1);
        }, Set.of(state.getDrive()));
    }

    protected Command shootFromStart(String startPath, double shakeSeconds) {
        return Commands.sequence(
                Commands.parallel(
                        follow(startPath),
                        intake(Intake.State.IDLE),
                        shooter(Shooter.State.HUB_TRACKING)),
                aim(),
                shooter(Shooter.State.SHOOTING),
                shake(shakeSeconds),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
