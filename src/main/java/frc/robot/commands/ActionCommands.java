package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.commands.AutoAlignToPoseCommand.AlignType;
import frc.robot.game.AllianceFlip;
import frc.robot.game.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TunableNumber;

public final class ActionCommands {
    private static final TunableNumber kFixedShotSpeed = new TunableNumber("FixedPos/RPS", 9.8);
    private static final TunableNumber kFixedShotHood = new TunableNumber("FixedPos/Hood", 0);
    private static final TunableNumber kFixedShotHoodFF = new TunableNumber("FixedPos/Hood FF", 0);

    public static Command shakeIntake(RobotState state) {
        return state.getSuperstructure().intakeCommand(intake -> Commands.sequence(
                intake.transitionCommand(Intake.State.SHAKE),
                Commands.waitSeconds(0.6),
                intake.transitionCommand(Intake.State.IDLE),
                Commands.waitSeconds(0.6))
                .repeatedly());
    }

    public static Command aimAtHub(RobotState state) {
        return alignToHub(state, AlignType.DEFAULT);
    }

    public static Command turnToHub(RobotState state) {
        return alignToHub(state, AlignType.ROTATION);
    }

    private static Command alignToHub(RobotState state, AlignType alignType) {
        return new DeferredCommand(() -> {
            Pose2d pose = state.getLatestFieldToRobot().getValue();
            Pose2d aimed = new Pose2d(pose.getX(), pose.getY(), state.getDrive().getAimRotationForHub());
            return new AutoAlignToPoseCommand(state.getDrive(), state, aimed, 1, alignType);
        }, Set.of(state.getDrive()));
    }

    public static Command aimAndShoot(RobotState state) {
        return state.getSuperstructure().shooterCommand(shooter -> Commands.sequence(
                Commands.runOnce(() -> shooter.requestTransition(Shooter.State.HUB_TRACKING)),
                Commands.runOnce(() -> shooter.requestTransition(Shooter.State.SHOOTING))));
    }

    public static Command shootOrPassBasedOnPos(RobotState state) {
        return state.getSuperstructure().shooterCommand(shooter -> new DeferredCommand(
                () -> shooter.transitionCommand(state.shouldShootHub() ? Shooter.State.SHOOTING : Shooter.State.PASSING),
                Set.of(shooter)));
    }

    public static Command trackBasedOnPos(RobotState state) {
        return state.getSuperstructure().shooterCommand(shooter -> new DeferredCommand(
                () -> shooter.transitionCommand(
                        state.shouldShootHub() ? Shooter.State.HUB_TRACKING : Shooter.State.PASS_TRACKING),
                Set.of(shooter)));
    }

    public static Command goToFixedPosAndShoot(RobotState state) {
        Set<Subsystem> requirements = new HashSet<>(Set.of(state.getDrive()));
        state.getSuperstructure().getShooter().ifPresent(requirements::add);

        return new DeferredCommand(() -> {
            Pose2d goalPose = AllianceFlip.forAlliance(FieldConstants.HUB_NEAR_FACE.transformBy(
                    new Transform2d(new Translation2d(Units.inchesToMeters(80), 0), new Rotation2d())));
            double speed = kFixedShotSpeed.get();
            double hoodPosition = kFixedShotHood.get();
            double hoodFF = kFixedShotHoodFF.get();

            return Commands.sequence(
                    new AutoAlignToPoseCommand(state.getDrive(), state, goalPose, 1),
                    Commands.runOnce(state.getSuperstructure().shooterAction(
                            shooter -> shooter.holdShot(speed, hoodPosition, hoodFF))));
        }, requirements);
    }

    private ActionCommands() {}
}
