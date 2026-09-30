package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DepotSideDepotMidHalfSweep extends CompetitionAuto {
    public DepotSideDepotMidHalfSweep(CompetitionSuperstructure robot) {
        super(robot, "Depot Side Depot Mid Half Sweep (GAME)",
                "Start Depot Side to Depot",
                "Depot Intaking",
                "Depot to Mid Under Trench",
                "Mid Depot Side Sweep",
                "Mid HP Side to Home HP");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                Commands.parallel(
                        follow("Start Depot Side to Depot"),
                        requestIntake(Intake.State.STOW),
                        requestShooter(Shooter.State.HUB_TRACKING)),
                Commands.parallel(
                        follow("Depot Intaking"),
                        requestIntake(Intake.State.INTAKE)),
                Commands.parallel(
                        requestIntake(Intake.State.STOW),
                        Commands.sequence(
                                requestShooter(Shooter.State.SHOOTING),
                                shake(4),
                                requestShooter(Shooter.State.HUB_TRACKING)),
                        Commands.sequence(
                                Commands.waitSeconds(1),
                                follow("Depot to Mid Under Trench"))),
                Commands.parallel(
                        follow("Mid Depot Side Sweep"),
                        requestIntake(Intake.State.INTAKE)),
                Commands.parallel(
                        follow("Mid HP Side to Home HP"),
                        requestIntake(Intake.State.STOW)),
                Commands.parallel(
                        requestIntake(Intake.State.STOW),
                        requestShooter(Shooter.State.SHOOTING),
                        shake(5.5)),
                requestShooter(Shooter.State.HUB_TRACKING));
    }
}
