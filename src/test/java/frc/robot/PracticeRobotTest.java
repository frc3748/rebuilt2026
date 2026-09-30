package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.autos.DepotSideToDepot;
import frc.robot.robots.practice.PracticeRobot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;

class PracticeRobotTest {
    private static RobotState state;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        state = new RobotState(new PracticeRobot());
        loop(25);
    }

    private static void loop(int times) {
        for (int i = 0; i < times; i++) {
            CommandScheduler.getInstance().run();
            state.updateLogger();
            state.updateSimulation();
        }
    }

    @Test
    void bootsWithOnlyADrivetrain() {
        assertEquals("Practice", state.getDefinition().name());
        assertTrue(state.getSuperstructure().getIntake().isEmpty());
        assertTrue(state.getSuperstructure().getShooter().isEmpty());
        assertTrue(state.getSuperstructure().subsystems().isEmpty());
        assertEquals(Drive.State.TRAVERSING, state.getDrive().getState());
    }

    @Test
    void usesItsOwnDriveConstants() {
        DriveConfig config = state.getDrive().getConfig();
        assertEquals(DriveConfig.GyroType.NAVX, config.gyro);
        assertEquals(DriveConfig.TurnSensor.SPARK_ABSOLUTE_ENCODER, config.turnSensor);
        assertEquals(3.5, config.maxSpeedMetersPerSec);
    }

    @Test
    void mechanismCommandsDoNothingWithoutTheMechanism() {
        assertTrue(ActionCommands.shakeIntake(state).getRequirements().isEmpty());
        assertTrue(ActionCommands.shootOrPassBasedOnPos(state).getRequirements().isEmpty());
        assertEquals(1, ActionCommands.goToFixedPosAndShoot(state).getRequirements().size());
    }

    @Test
    void runsTheSharedAutosOnItsDrivetrain() {
        for (AutoRoutine auto : Autos.all(state)) {
            assertFalse(auto.build().getName().endsWith("(FAILED)"), auto.name());
        }

        Command auto = new DepotSideToDepot(state).build();
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().schedule(auto);
        loop(50);
        assertTrue(auto.isScheduled());
        assertTrue(state.getLatestFieldToRobot().getValue().getTranslation().getNorm() > 0.5);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }
}
