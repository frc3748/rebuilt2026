package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.shooter.Shooter;

public class HpSideToHp extends CompetitionAuto {
    public HpSideToHp(CompetitionSuperstructure robot) {
        super(robot, "HP Side To HP (GAME)",
                "Start HP Side to Home HP",
                "Home HP to HP");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shootFromStart("Start HP Side to Home HP", 4),
                Commands.parallel(
                        follow("Home HP to HP"),
                        shooter(Shooter.State.HUB_TRACKING)),
                aim(),
                shooter(Shooter.State.SHOOTING),
                Commands.waitSeconds(8),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
