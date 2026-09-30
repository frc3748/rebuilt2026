package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class DepotOnlyStarting8 extends PathAuto {
    public DepotOnlyStarting8(RobotState state) {
        super(state, "Depot Only Starting 8 (GAME)", "Start Depot Side to Home Depot");
    }

    @Override
    protected Command routine() {
        return shootFromStart("Start Depot Side to Home Depot", 10);
    }
}
