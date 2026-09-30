package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class HpSideQuickShoot extends PathAuto {
    public HpSideQuickShoot(RobotState state) {
        super(state, "HP Side Quick Shoot (GAME)",
                "Start HP Side to Mid Intake",
                "Mid Intake to Start HP Side",
                "Start HP Side to Mid Intake Second");
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.INTAKE),
                Commands.waitSeconds(0.25),
                follow("Start HP Side to Mid Intake"),
                Commands.parallel(
                        follow("Mid Intake to Start HP Side"),
                        Commands.sequence(Commands.waitSeconds(0.2), intake(Intake.State.IDLE))),
                turn(),
                shooter(Shooter.State.SHOOTING),
                shake(5),
                intake(Intake.State.IDLE),
                shooter(Shooter.State.HUB_TRACKING),
                Commands.parallel(
                        follow("Start HP Side to Mid Intake Second"),
                        Commands.sequence(Commands.waitSeconds(0.6), intake(Intake.State.INTAKE))),
                nudge(1),
                Commands.parallel(
                        follow("Mid Intake to Start HP Side"),
                        Commands.sequence(Commands.waitSeconds(0.2), intake(Intake.State.IDLE))),
                turn(),
                shooter(Shooter.State.SHOOTING),
                shake(8),
                intake(Intake.State.IDLE),
                shooter(Shooter.State.HUB_TRACKING));
    }
}
