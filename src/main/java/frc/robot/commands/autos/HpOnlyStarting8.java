package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class HpOnlyStarting8 extends PathAuto {
    public HpOnlyStarting8(RobotState state) {
        super(state, "HP Only Starting 8 (GAME)", "Start HP Side to Home HP");
    }

    @Override
    protected Command routine() {
        return shootFromStart("Start HP Side to Home HP", 10);
    }
}
