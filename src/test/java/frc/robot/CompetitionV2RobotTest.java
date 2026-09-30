package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.competitionv2.CompetitionV2Drive;
import frc.robot.robots.competitionv2.CompetitionV2Robot;
import frc.robot.subsystems.drive.Drive;

class CompetitionV2RobotTest {
    private static RobotState state;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        state = new RobotState(new CompetitionV2Robot());
        for (int i = 0; i < 25; i++) {
            CommandScheduler.getInstance().run();
            state.updateLogger();
            state.updateSimulation();
        }
    }

    @Test
    void reusesTheCompetitionMechanismsAndCameras() {
        assertTrue(state.getSuperstructure().getShooter().isPresent());
        assertTrue(state.getSuperstructure().getIntake().isPresent());
        assertEquals(2, state.getDefinition().cameras().length);
    }

    @Test
    void drivesOnItsOwnModules() {
        assertInstanceOf(CompetitionV2Drive.class, state.getDrive().getConfig());
        assertEquals(Drive.State.TRAVERSING, state.getDrive().getState());
    }
}
