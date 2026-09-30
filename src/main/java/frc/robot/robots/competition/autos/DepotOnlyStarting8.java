package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.competition.CompetitionSuperstructure;

public class DepotOnlyStarting8 extends CompetitionAuto {
    public DepotOnlyStarting8(CompetitionSuperstructure robot) {
        super(robot, "Depot Only Starting 8 (GAME)", "Start Depot Side to Home Depot");
    }

    @Override
    protected Command routine() {
        return shootFromStart("Start Depot Side to Home Depot", 10);
    }
}
