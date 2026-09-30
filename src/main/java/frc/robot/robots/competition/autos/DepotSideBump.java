package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class DepotSideBump extends CompetitionAuto {
    public DepotSideBump(CompetitionSuperstructure robot) {
        super(robot, "Depot Side Bump (GAME)",
                "Start Depot to Intake Blair",
                "Mid Depot to Intake Bump",
                "Start Depot Side to Intake Bump Second");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.IDLE),
                Commands.parallel(
                        follow("Start Depot to Intake Blair"),
                        Commands.sequence(Commands.waitSeconds(1.3), intake(Intake.State.INTAKE))),
                flywheel(Flywheel.State.SHOOT),
                follow("Mid Depot to Intake Bump"),
                Commands.parallel(
                        turn(),
                        Commands.sequence(
                                intake(Intake.State.IDLE),
                                shooter(Shooter.State.SHOOTING))),
                shake(3),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(1),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Start Depot Side to Intake Bump Second"),
                        Commands.sequence(Commands.waitSeconds(1.3), intake(Intake.State.INTAKE))),
                flywheel(Flywheel.State.SHOOT),
                follow("Mid Depot to Intake Bump"),
                Commands.parallel(
                        turn(),
                        Commands.sequence(
                                intake(Intake.State.IDLE),
                                shooter(Shooter.State.SHOOTING))),
                shake(11),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(1),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
