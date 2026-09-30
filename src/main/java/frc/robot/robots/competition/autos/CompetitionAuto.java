package frc.robot.robots.competition.autos;

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
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.game.AllianceFlip;
import frc.robot.robots.competition.ActionCommands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public abstract class CompetitionAuto extends AutoRoutine {
    protected final CompetitionSuperstructure robot;
    private Map<String, PathPlannerPath> paths = Map.of();

    protected CompetitionAuto(CompetitionSuperstructure robot, String name, String... pathNames) {
        super(name, pathNames);
        this.robot = robot;
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
        robot.getDrive().setPose(AllianceFlip.forAlliance(start.get()));
    }

    protected Command follow(String pathName) {
        return AutoBuilder.followPath(paths.get(pathName));
    }

    protected Command intake(Intake.State state) {
        return robot.getIntake().transitionCommand(state);
    }

    protected Command shooter(Shooter.State state) {
        return robot.getShooter().transitionCommand(state);
    }

    protected Command flywheel(Flywheel.State state) {
        return robot.getFlywheel().transitionCommand(state);
    }

    protected Command requestIntake(Intake.State state) {
        return robot.getIntake().transitionCommand(state, false);
    }

    protected Command requestShooter(Shooter.State state) {
        return robot.getShooter().transitionCommand(state, false);
    }

    protected Command shake(double seconds) {
        return ActionCommands.shakeIntake(robot).withTimeout(seconds);
    }

    protected Command aim() {
        return ActionCommands.aimAtHub(robot);
    }

    protected Command turn() {
        return ActionCommands.turnToHub(robot);
    }

    protected Command nudge(double metersForward) {
        return new DeferredCommand(() -> {
            Pose2d target = robot.state().getLatestFieldToRobot().getValue()
                    .plus(new Transform2d(new Translation2d(metersForward, 0), new Rotation2d()));
            return new AutoAlignToPoseCommand(robot.getDrive(), robot.state(), target, 1);
        }, Set.of(robot.getDrive()));
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
