package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.competition.CompetitionSuperstructure;

public class HpOnlyStarting8 extends CompetitionAuto {
    public HpOnlyStarting8(CompetitionSuperstructure robot) {
        super(robot, "HP Only Starting 8 (GAME)", "Start HP Side to Home HP");
    }

    @Override
    protected Command routine() {
        return shootFromStart("Start HP Side to Home HP", 10);
    }
}
