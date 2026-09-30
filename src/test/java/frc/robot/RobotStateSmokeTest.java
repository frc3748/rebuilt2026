package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

class RobotStateSmokeTest {
    @Test
    void constructsAndRunsPeriodicLoops() {
        assertTrue(HAL.initialize(500, 0));

        RobotState state = new RobotState();
        for (int i = 0; i < 25; i++) {
            CommandScheduler.getInstance().run();
            state.updateLogger();
            state.updateSimulation();
        }

        state.getShooter().requestTransition(Shooter.State.SHOOTING);
        state.getIntake().requestTransition(Intake.State.INTAKE);
        for (int i = 0; i < 25; i++) {
            CommandScheduler.getInstance().run();
        }
        assertEquals(Shooter.State.SHOOTING, state.getShooter().getState());
        assertEquals(Intake.State.INTAKE, state.getIntake().getState());
    }
}
