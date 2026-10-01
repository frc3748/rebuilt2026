package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ActionCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Controls {
    protected final CommandXboxController driver = new CommandXboxController(0);
    protected final CommandXboxController operator = new CommandXboxController(1);

    public Controls() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void bind(RobotState state) {
        Superstructure robot = state.getSuperstructure();
        bindDrive(state);
        robot.getIntake().ifPresent(intake -> bindIntake(state, intake));
        robot.getShooter().ifPresent(shooter -> bindShooter(state, shooter));
        bindOperator(state, robot);
    }

    protected void bindDrive(RobotState state) {
        Drive drive = state.getDrive();
        if (DriverStation.getMatchType() == MatchType.None) {
            headingResetButton().onTrue(Commands.runOnce(drive::zeroHeading, drive).ignoringDisable(true));
        }
        slowButton()
                .onTrue(drive.transitionCommand(Drive.State.SLOW))
                .onFalse(drive.transitionCommand(Drive.State.TRAVERSING));
        driver.povLeft().onTrue(drive.transitionCommand(Drive.State.TRAVERSING_AT_ANGLE));
        driver.povRight().onTrue(drive.transitionCommand(Drive.State.TRAVERSING));
        driver.povUp().whileTrue(ActionCommands.turnToHub(state));
    }

    protected Trigger headingResetButton() {
        return driver.povDown();
    }

    protected Trigger slowButton() {
        return driver.rightBumper();
    }

    protected void bindIntake(RobotState state, Intake intake) {
        driver.leftTrigger(0.5)
                .onTrue(intake.transitionCommand(Intake.State.INTAKE))
                .onFalse(intake.transitionCommand(Intake.State.IDLE));
        driver.leftBumper().onTrue(intake.transitionCommand(Intake.State.STOW));
        driver.a()
                .onTrue(Commands.runOnce(() -> intake.requestTransition(Intake.State.OUTAKE)))
                .onFalse(Commands.runOnce(() -> intake.requestTransition(Intake.State.IDLE)));
        driver.b().whileTrue(ActionCommands.shakeIntake(state));
    }

    protected void bindShooter(RobotState state, Shooter shooter) {
        Drive drive = state.getDrive();
        driver.rightTrigger(0.5)
                .onTrue(Commands.sequence(
                        Commands.runOnce(drive::stopWithX),
                        ActionCommands.shootOrPassBasedOnPos(state)))
                .onFalse(ActionCommands.trackBasedOnPos(state));
        driver.a()
                .onTrue(Commands.runOnce(() -> shooter.requestTransition(Shooter.State.OUTTAKE)))
                .onFalse(ActionCommands.trackBasedOnPos(state));
        driver.x()
                .whileTrue(ActionCommands.goToFixedPosAndShoot(state))
                .onFalse(Commands.runOnce(shooter::releaseShot));
    }

    protected void bindOperator(RobotState state, Superstructure robot) {
        operator.leftStick().onTrue(Commands.runOnce(robot::clearOverrides));

        bindChord(operator.rightStick(),
                robot.shooterAction(Shooter::releaseFeed),
                robot.intakeAction(Intake::clearOverride),
                robot.shooterAction(Shooter::releaseShot));

        bindChord(operator.leftTrigger(0.5),
                robot.shooterAction(Shooter::stopFeed),
                robot.intakeAction(intake -> intake.setOverride(intake::rollIn)),
                robot.shooterAction(shooter -> shooter.holdShot(state.getCurrentHubSetpoint(), true)));

        bindChord(operator.rightTrigger(0.5),
                robot.shooterAction(Shooter::stopFeed),
                robot.intakeAction(intake -> intake.setOverride(intake::rollOut)),
                robot.shooterAction(shooter -> shooter.holdShot(state.getCurrentPassSetpoint(), true)));

        bindChord(operator.leftBumper(),
                robot.shooterAction(Shooter::forceFeed),
                robot.intakeAction(intake -> intake.setOverride(Intake.State.STOW)),
                robot.shooterAction(shooter -> shooter.holdShot(state.getCurrentHubSetpoint(), false)));

        bindChord(operator.rightBumper(),
                robot.shooterAction(Shooter::reverseFeed),
                robot.intakeAction(intake -> intake.setOverride(Intake.State.INTAKE)),
                robot.shooterAction(shooter -> shooter.holdShot(state.getCurrentPassSetpoint(), false)));
    }

    protected void bindChord(Trigger button, Runnable feedAction, Runnable intakeAction, Runnable shooterAction) {
        button.onTrue(Commands.runOnce(() -> {
            if (operator.x().getAsBoolean()) {
                feedAction.run();
            } else if (operator.b().getAsBoolean()) {
                intakeAction.run();
            } else if (operator.a().getAsBoolean()) {
                shooterAction.run();
            }
        }));
    }

    public Command pulse(double seconds) {
        return Commands.repeatingSequence(
                Commands.runOnce(() -> setRumble(1.0)), Commands.waitSeconds(0.15),
                Commands.runOnce(() -> setRumble(0.0)), Commands.waitSeconds(0.15))
                .withTimeout(seconds)
                .finallyDo(() -> setRumble(0.0));
    }

    public Command driverBuzz(double seconds) {
        return Commands.startEnd(() -> driver.getHID().setRumble(RumbleType.kRightRumble, 0.6),
                () -> driver.getHID().setRumble(RumbleType.kRightRumble, 0.0)).withTimeout(seconds);
    }

    public Command rumble(double seconds) {
        return Commands.startEnd(() -> setRumble(1.0), () -> setRumble(0.0)).withTimeout(seconds);
    }

    private void setRumble(double value) {
        driver.getHID().setRumble(RumbleType.kBothRumble, value);
        operator.getHID().setRumble(RumbleType.kBothRumble, value);
    }

    public CommandXboxController driver() {
        return driver;
    }

    public CommandXboxController operator() {
        return operator;
    }
}
