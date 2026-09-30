package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class CenterOnlyStarting8 extends PathAuto {
    public CenterOnlyStarting8(RobotState state) {
        super(state, "Center Only Starting 8 (GAME)", "Start Center to Home Center");
    }

    @Override
    protected Command routine() {
        return shootFromStart("Start Center to Home Center", 10);
    }
}
