package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DepotSideCircuitShoot extends PathAuto {
    public DepotSideCircuitShoot(RobotState state) {
        super(state, "Depot Side Circut Shoot (GAME)",
                "Start Depot Side to Mid Intake Circut",
                "Start Depot Side to Mid Intake Circut Second");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(0.25),
                Commands.parallel(
                        follow("Start Depot Side to Mid Intake Circut"),
                        Commands.sequence(Commands.waitSeconds(1.4), intake(Intake.State.INTAKE)),
                        Commands.sequence(Commands.waitSeconds(3.4), intake(Intake.State.IDLE))),
                aim(),
                shooter(Shooter.State.SHOOTING),
                shake(5),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Start Depot Side to Mid Intake Circut Second"),
                        Commands.sequence(Commands.waitSeconds(1.8), intake(Intake.State.INTAKE)),
                        Commands.sequence(Commands.waitSeconds(4), intake(Intake.State.IDLE))),
                aim(),
                shooter(Shooter.State.SHOOTING),
                shake(5),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
