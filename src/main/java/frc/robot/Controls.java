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
import frc.robot.util.cockpit.Cockpit;

public class Controls {
    private static final double kTriggerThreshold = 0.5;
    private static final double kShotSpeedStep = 0.02;

    protected final CommandXboxController driver = new CommandXboxController(0);
    protected final CommandXboxController operator = new CommandXboxController(1);

    public Controls() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void bind(RobotState state) {
        Superstructure robot = state.getSuperstructure();
        bindDriver(state);
        robot.getIntake().ifPresent(intake -> bindIntake(state, intake));
        robot.getShooter().ifPresent(shooter -> bindShooter(state, shooter));
        bindOperator(robot);
    }

    protected void bindDriver(RobotState state) {
        Drive drive = state.getDrive();
        driverControl("RB RT", "Align to shoot", driver.rightBumper().or(driver.rightTrigger(kTriggerThreshold)))
                .onTrue(drive.transitionCommand(Drive.State.TRAVERSING_AT_ANGLE))
                .onFalse(drive.transitionCommand(Drive.State.TRAVERSING));
        driverControl("LB LT", "Turbo", driver.leftBumper().or(driver.leftTrigger(kTriggerThreshold)))
                .whileTrue(drive.turbo());
        if (DriverStation.getMatchType() == MatchType.None) {
            driverControl("Down", "Zero heading", driver.povDown())
                    .onTrue(Commands.runOnce(drive::zeroHeading, drive).ignoringDisable(true));
        }
    }

    protected void bindIntake(RobotState state, Intake intake) {
        operatorControl("LB", "Intake down", operator.leftBumper())
                .onTrue(intake.transitionCommand(Intake.State.IDLE));
        operatorControl("RB", "Intake up", operator.rightBumper())
                .onTrue(intake.transitionCommand(Intake.State.STOW));
        operatorControl("LT", "Intake", operator.leftTrigger(kTriggerThreshold))
                .onTrue(intake.transitionCommand(Intake.State.INTAKE))
                .onFalse(intake.transitionCommand(Intake.State.IDLE));
        operatorControl("B", "Shake", operator.b())
                .whileTrue(ActionCommands.shakeIntake(state))
                .onFalse(intake.transitionCommand(Intake.State.IDLE));
        operatorControl("A", "Eject", operator.a())
                .onTrue(intake.transitionCommand(Intake.State.OUTAKE))
                .onFalse(intake.transitionCommand(Intake.State.IDLE));
    }

    protected void bindShooter(RobotState state, Shooter shooter) {
        operatorControl("RT", "Shoot", operator.rightTrigger(kTriggerThreshold))
                .onTrue(ActionCommands.shootOrPassBasedOnPos(state))
                .onFalse(ActionCommands.trackBasedOnPos(state));
        operator.a()
                .onTrue(Commands.runOnce(() -> shooter.requestTransition(Shooter.State.OUTTAKE)))
                .onFalse(ActionCommands.trackBasedOnPos(state));
        operatorControl("Y", "Hub-front shot", operator.y())
                .whileTrue(ActionCommands.fixedShot(state));
        operatorControl("X", "Unjam", operator.x())
                .onTrue(Commands.runOnce(shooter::reverseFeed))
                .onFalse(Commands.runOnce(shooter::releaseFeed));
        operatorControl("Up", "Shot speed +2%", operator.povUp())
                .onTrue(Commands.runOnce(() -> shooter.adjustMultiplier(kShotSpeedStep)));
        operatorControl("Down", "Shot speed -2%", operator.povDown())
                .onTrue(Commands.runOnce(() -> shooter.adjustMultiplier(-kShotSpeedStep)));
        operatorControl("Back", "Reset shot speed", operator.back())
                .onTrue(Commands.runOnce(shooter::resetMultiplier));
    }

    protected void bindOperator(Superstructure robot) {
        operatorControl("Start", "Clear overrides", operator.start())
                .onTrue(Commands.runOnce(robot::clearOverrides));
    }

    private Trigger driverControl(String inputs, String label, Trigger trigger) {
        Cockpit.control("driver", inputs, label);
        return trigger;
    }

    private Trigger operatorControl(String inputs, String label, Trigger trigger) {
        Cockpit.control("operator", inputs, label);
        return trigger;
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
