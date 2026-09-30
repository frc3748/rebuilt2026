package frc.robot.robots.competition;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotState;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.AutoAlignToPoseCommand.AlignType;
import frc.robot.game.AllianceFlip;
import frc.robot.game.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.TunableNumber;

public final class ActionCommands {
    private static final TunableNumber kFixedShotSpeed = new TunableNumber("FixedPos/RPS", 9.8);
    private static final TunableNumber kFixedShotHood = new TunableNumber("FixedPos/Hood", 0);
    private static final TunableNumber kFixedShotHoodFF = new TunableNumber("FixedPos/Hood FF", 0);

    public static Command shakeIntake(CompetitionSuperstructure robot) {
        Intake intake = robot.getIntake();
        return Commands.sequence(
                intake.transitionCommand(Intake.State.SHAKE),
                Commands.waitSeconds(0.6),
                intake.transitionCommand(Intake.State.IDLE),
                Commands.waitSeconds(0.6))
                .repeatedly();
    }

    public static Command aimAtHub(CompetitionSuperstructure robot) {
        return alignToHub(robot.state(), AlignType.DEFAULT);
    }

    public static Command turnToHub(CompetitionSuperstructure robot) {
        return alignToHub(robot.state(), AlignType.ROTATION);
    }

    private static Command alignToHub(RobotState state, AlignType alignType) {
        return new DeferredCommand(() -> {
            Pose2d pose = state.getLatestFieldToRobot().getValue();
            Pose2d aimed = new Pose2d(pose.getX(), pose.getY(), state.getDrive().getAimRotationForHub());
            return new AutoAlignToPoseCommand(state.getDrive(), state, aimed, 1, alignType);
        }, Set.of(state.getDrive()));
    }

    public static Command aimAndShoot(CompetitionSuperstructure robot) {
        Shooter shooter = robot.getShooter();
        return Commands.sequence(
                Commands.runOnce(() -> shooter.requestTransition(Shooter.State.HUB_TRACKING)),
                Commands.runOnce(() -> shooter.requestTransition(Shooter.State.SHOOTING)));
    }

    public static Command shootOrPassBasedOnPos(CompetitionSuperstructure robot) {
        Shooter shooter = robot.getShooter();
        return new DeferredCommand(
                () -> shooter.transitionCommand(robot.state().shouldShootHub() ? Shooter.State.SHOOTING : Shooter.State.PASSING),
                Set.of(shooter));
    }

    public static Command trackBasedOnPos(CompetitionSuperstructure robot) {
        Shooter shooter = robot.getShooter();
        return new DeferredCommand(
                () -> shooter.transitionCommand(
                        robot.state().shouldShootHub() ? Shooter.State.HUB_TRACKING : Shooter.State.PASS_TRACKING),
                Set.of(shooter));
    }

    public static Command goToFixedPosAndShoot(CompetitionSuperstructure robot) {
        RobotState state = robot.state();
        return new DeferredCommand(() -> {
            Pose2d goalPose = AllianceFlip.forAlliance(FieldConstants.HUB_NEAR_FACE.transformBy(
                    new Transform2d(new Translation2d(Units.inchesToMeters(80), 0), new Rotation2d())));
            double speed = kFixedShotSpeed.get();
            double hoodPosition = kFixedShotHood.get();
            double hoodFF = kFixedShotHoodFF.get();
            Flywheel flywheel = robot.getFlywheel();
            Hood hood = robot.getHood();

            return Commands.sequence(
                    new AutoAlignToPoseCommand(state.getDrive(), state, goalPose, 1),
                    Commands.runOnce(() -> {
                        flywheel.setOverride(() -> flywheel.spin(speed));
                        hood.setOverride(() -> hood.setPos(hoodPosition, hoodFF));
                    }));
        }, Set.of(state.getDrive(), robot.getShooter()));
    }

    private ActionCommands() {}
}
