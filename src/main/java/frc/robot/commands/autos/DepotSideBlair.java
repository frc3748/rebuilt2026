package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DepotSideBlair extends PathAuto {
    public DepotSideBlair(RobotState state) {
        super(state, "Depot Side Blair (GAME)",
                "Start Depot to Intake Blair",
                "Mid Depot to Intake Depot Side Blair",
                "Start Depot to Intake Second Blair",
                "Mid Depot to Intake Depot Side Second Blair");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.IDLE),
                Commands.parallel(
                        follow("Start Depot to Intake Blair"),
                        Commands.sequence(Commands.waitSeconds(1.3), intake(Intake.State.INTAKE))),
                intake(Intake.State.IDLE),
                spinUp(),
                follow("Mid Depot to Intake Depot Side Blair"),
                Commands.parallel(
                        turn(),
                        shooter(Shooter.State.SHOOTING)),
                shake(3),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(1),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Start Depot to Intake Second Blair"),
                        Commands.sequence(Commands.waitSeconds(1.3), intake(Intake.State.INTAKE))),
                intake(Intake.State.IDLE),
                spinUp(),
                follow("Mid Depot to Intake Depot Side Second Blair"),
                Commands.parallel(
                        turn(),
                        shooter(Shooter.State.SHOOTING)),
                shake(11),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(1),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
