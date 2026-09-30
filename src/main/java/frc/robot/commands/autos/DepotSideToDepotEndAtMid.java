package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DepotSideToDepotEndAtMid extends PathAuto {
    public DepotSideToDepotEndAtMid(RobotState state) {
        super(state, "Depot Side To Depot End at Mid (GAME)",
                "Start Depot Side to Depot",
                "Depot Intaking",
                "Depot to Mid Under Trench",
                "Mid Depot Side Half Sweep");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                Commands.parallel(
                        follow("Start Depot Side to Depot"),
                        intake(Intake.State.IDLE)),
                aim(),
                shooter(Shooter.State.SHOOTING),
                shake(3.5),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Depot Intaking"),
                        intake(Intake.State.INTAKE)),
                intake(Intake.State.IDLE),
                Commands.waitSeconds(2),
                follow("Depot to Mid Under Trench"),
                Commands.parallel(
                        follow("Mid Depot Side Half Sweep"),
                        intake(Intake.State.INTAKE),
                        shooter(Shooter.State.PASS_TRACKING)),
                intake(Intake.State.IDLE));
    }
}
