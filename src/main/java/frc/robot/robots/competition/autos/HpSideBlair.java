package frc.robot.robots.competition.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.competition.CompetitionSuperstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class HpSideBlair extends CompetitionAuto {
    public HpSideBlair(CompetitionSuperstructure robot) {
        super(robot, "HP Side Blair (GAME)",
                "Start HP to Intake Blair",
                "Mid HP to Intake HP Side Blair",
                "Start HP to Intake Second Blair",
                "Mid HP to Intake HP Side Second Blair");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.IDLE),
                Commands.parallel(
                        follow("Start HP to Intake Blair"),
                        Commands.sequence(Commands.waitSeconds(1.3), intake(Intake.State.INTAKE))),
                intake(Intake.State.IDLE),
                flywheel(Flywheel.State.SHOOT),
                follow("Mid HP to Intake HP Side Blair"),
                Commands.parallel(
                        turn(),
                        shooter(Shooter.State.SHOOTING)),
                shake(3.5),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(0.5),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Start HP to Intake Second Blair"),
                        Commands.sequence(Commands.waitSeconds(1.3), intake(Intake.State.INTAKE))),
                intake(Intake.State.IDLE),
                flywheel(Flywheel.State.SHOOT),
                follow("Mid HP to Intake HP Side Second Blair"),
                Commands.parallel(
                        turn(),
                        shooter(Shooter.State.SHOOTING)),
                shake(11.5),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(0.5),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
