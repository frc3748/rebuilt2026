package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DepotSideToDepot extends PathAuto {
    public DepotSideToDepot(RobotState state) {
        super(state, "Depot Side To Depot (GAME)",
                "Start Depot Side to Home Depot",
                "Home Depot to Depot",
                "Depot Intaking",
                "Depot Intaking to Depot");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                Commands.parallel(
                        follow("Start Depot Side to Home Depot"),
                        intake(Intake.State.IDLE),
                        shooter(Shooter.State.HUB_TRACKING)),
                follow("Home Depot to Depot"),
                Commands.parallel(
                        follow("Depot Intaking"),
                        intake(Intake.State.INTAKE)),
                follow("Depot Intaking to Depot"),
                turn().withTimeout(2),
                Commands.parallel(
                        intake(Intake.State.IDLE),
                        shooter(Shooter.State.SHOOTING)),
                shake(14),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
