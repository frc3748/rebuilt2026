package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DepotSideQuickShoot extends CompetitionAuto {
    public DepotSideQuickShoot(CompetitionSuperstructure robot) {
        super(robot, "Depot Side Quick Shoot (GAME)",
                "Start Depot Side to Mid Intake",
                "Mid Intake to Start Depot Side",
                "Start Depot Side to Mid Intake Second",
                "Mid Intake to Start Depot Side Second");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.INTAKE),
                follow("Start Depot Side to Mid Intake"),
                Commands.parallel(
                        follow("Mid Intake to Start Depot Side"),
                        Commands.sequence(Commands.waitSeconds(0.2), intake(Intake.State.IDLE))),
                turn(),
                shooter(Shooter.State.SHOOTING),
                shake(3),
                intake(Intake.State.IDLE),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Start Depot Side to Mid Intake Second"),
                        Commands.sequence(Commands.waitSeconds(0.6), intake(Intake.State.INTAKE))),
                Commands.parallel(
                        follow("Mid Intake to Start Depot Side Second"),
                        Commands.sequence(Commands.waitSeconds(0.2), intake(Intake.State.IDLE))),
                turn(),
                shooter(Shooter.State.SHOOTING),
                shake(8),
                intake(Intake.State.IDLE),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
