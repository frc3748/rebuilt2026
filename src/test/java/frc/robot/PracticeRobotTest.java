package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.practice.PracticeRobot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;

class PracticeRobotTest {
    private static RobotState state;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        state = new RobotState(new PracticeRobot());
        for (int i = 0; i < 25; i++) {
            CommandScheduler.getInstance().run();
            state.updateLogger();
            state.updateSimulation();
        }
    }

    @Test
    void bootsWithOnlyADrivetrain() {
        assertEquals("Practice", state.getDefinition().name());
        assertTrue(state.getSuperstructure().subsystems().isEmpty());
        assertTrue(state.getSuperstructure().autos().isEmpty());
        assertEquals(Drive.State.TRAVERSING, state.getDrive().getState());
    }

    @Test
    void usesItsOwnDriveConstants() {
        DriveConfig config = state.getDrive().getConfig();
        assertEquals(DriveConfig.GyroType.NAVX, config.gyro);
        assertEquals(DriveConfig.TurnSensor.SPARK_ABSOLUTE_ENCODER, config.turnSensor);
        assertEquals(3.5, config.maxSpeedMetersPerSec);
    }
}
