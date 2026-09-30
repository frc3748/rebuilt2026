package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robots.comp.CompDrive;
import frc.robot.subsystems.drive.HeadingLock;

class HeadingLockTest {
    private Rotation2d heading = Rotation2d.kZero;
    private double yawRate;
    private final HeadingLock lock = new HeadingLock(new CompDrive(), () -> heading, () -> yawRate);

    @Test
    void waitsForTheRobotToStopTurningBeforeLocking() {
        yawRate = 2.0;
        heading = Rotation2d.fromDegrees(30);
        assertEquals(0.0, lock.hold());
        assertFalse(lock.isLocked());

        yawRate = 0.0;
        heading = Rotation2d.fromDegrees(40);
        assertEquals(0.0, lock.hold());
        assertTrue(lock.isLocked());
    }

    @Test
    void turnsBackTowardTheLockedHeadingWhenItDrifts() {
        heading = Rotation2d.fromDegrees(90);
        lock.hold();

        heading = Rotation2d.fromDegrees(85);
        assertTrue(lock.hold() > 0);

        heading = Rotation2d.fromDegrees(95);
        assertTrue(lock.hold() < 0);
    }

    @Test
    void locksAcrossTheWrapAround() {
        heading = Rotation2d.fromDegrees(179);
        lock.hold();

        heading = Rotation2d.fromDegrees(-178);
        assertTrue(lock.hold() < 0);
    }

    @Test
    void ignoresErrorInsideTheTolerance() {
        lock.hold();
        heading = Rotation2d.fromDegrees(0.5);
        assertEquals(0.0, lock.hold());
    }

    @Test
    void releaseForgetsTheLockedHeading() {
        lock.hold();
        lock.release();
        heading = Rotation2d.fromDegrees(120);
        assertEquals(0.0, lock.hold());
        heading = Rotation2d.fromDegrees(118);
        assertTrue(lock.hold() > 0);
    }
}
