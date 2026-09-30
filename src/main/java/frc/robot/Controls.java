package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

public class Controls {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public void bindDrive(Drive drive) {
        if (DriverStation.getMatchType() == MatchType.None) {
            driver.povDown().onTrue(Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive)
                    .ignoringDisable(true));
        }
        driver.rightBumper()
                .onTrue(drive.transitionCommand(Drive.State.SLOW))
                .onFalse(drive.transitionCommand(Drive.State.TRAVERSING));
        driver.povLeft().onTrue(drive.transitionCommand(Drive.State.TRAVERSING_AT_ANGLE));
        driver.povRight().onTrue(drive.transitionCommand(Drive.State.TRAVERSING));
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
