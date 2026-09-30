package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.comp.CompDrive;
import frc.robot.robots.comp.CompRobot;
import frc.robot.robots.secondary.SecondaryDrive;
import frc.robot.robots.secondary.SecondaryRobot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.intake.IntakeComp;
import frc.robot.subsystems.shooter.ShooterComp;

class SecondaryRobotTest {
    private static RobotState state;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        state = new RobotState(new SecondaryRobot());
        for (int i = 0; i < 25; i++) {
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    @Test
    void reusesTheCompMechanismsAndCameras() {
        assertInstanceOf(ShooterComp.class, state.getSuperstructure().getShooter().orElseThrow());
        assertInstanceOf(IntakeComp.class, state.getSuperstructure().getIntake().orElseThrow());
        assertEquals(new CompRobot().cameras().length, state.getDefinition().cameras().length);
    }

    @Test
    void overridesOnlyItsModuleOffsets() {
        DriveConfig secondary = state.getDrive().getConfig();
        DriveConfig comp = new CompDrive();
        assertInstanceOf(SecondaryDrive.class, secondary);
        assertEquals(comp.driveReduction, secondary.driveReduction);
        assertEquals(comp.maxSpeedMetersPerSec, secondary.maxSpeedMetersPerSec);
        assertNotEquals(comp.frontLeft.zeroRotation(), secondary.frontLeft.zeroRotation());
        assertEquals(Drive.State.TRAVERSING, state.getDrive().getState());
    }
}
