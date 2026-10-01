package frc.robot.robots.practice;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;

public class PracticeControls extends Controls {
    @Override
    protected Trigger headingResetButton() {
        return driver.rightBumper();
    }

    @Override
    protected Trigger slowButton() {
        return driver.leftBumper();
    }
}
