package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.competition.CompetitionSuperstructure;

public class CenterOnlyStarting8 extends CompetitionAuto {
    public CenterOnlyStarting8(CompetitionSuperstructure robot) {
        super(robot, "Center Only Starting 8 (GAME)", "Start Center to Home Center");
    }

    @Override
    protected Command routine() {
        return shootFromStart("Start Center to Home Center", 10);
    }
}
