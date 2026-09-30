package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class HpSideToHpEndAtMid extends CompetitionAuto {
    public HpSideToHpEndAtMid(CompetitionSuperstructure robot) {
        super(robot, "HP Side To HP End at Mid (GAME)",
                "Start HP Side to HP",
                "HP to Mid",
                "Mid HP Side Half Sweep");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                Commands.parallel(
                        follow("Start HP Side to HP"),
                        intake(Intake.State.IDLE)),
                aim(),
                shooter(Shooter.State.SHOOTING),
                shake(4),
                shooter(Shooter.State.HUB_TRACKING),
                follow("HP to Mid"),
                Commands.parallel(
                        follow("Mid HP Side Half Sweep"),
                        intake(Intake.State.INTAKE),
                        shooter(Shooter.State.PASS_TRACKING)),
                intake(Intake.State.IDLE));
    }
}
