package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class CollisionDetectorTest {
    private static final double kGravity = 9.80665;
    private static final double kDt = 0.02;

    private double time;

    private void feed(CollisionDetector detector, double wheelSpeed, double imuGs, double pitchRadians, boolean connected) {
        time += kDt;
        detector.update(new ChassisSpeeds(wheelSpeed, 0.0, 0.0), connected, imuGs, 0.0, pitchRadians, 0.0, time, kDt);
    }

    @Test
    void hardAccelerationTheWheelsExplainIsNotAHit() {
        CollisionDetector detector = new CollisionDetector();
        double accel = 0.8 * kGravity;
        for (int i = 1; i <= 25; i++) {
            feed(detector, accel * kDt * i, 0.8, 0.0, true);
        }
        assertFalse(detector.isHit());
        assertFalse(detector.isUpset());
        assertEquals(0, detector.events());
        assertEquals(1.0, detector.visionStdDevScale());
    }

    @Test
    void aJoltTheWheelsDontFeelTrustsVisionForASecond() {
        CollisionDetector detector = new CollisionDetector();
        for (int i = 0; i < 10; i++) {
            feed(detector, 2.0, 0.0, 0.0, true);
        }
        feed(detector, 2.0, 2.5, 0.0, true);
        assertTrue(detector.isHit());
        assertEquals(1, detector.events());
        assertTrue(detector.visionStdDevScale() < 1.0);
        for (int i = 0; i < 40; i++) {
            feed(detector, 2.0, 0.0, 0.0, true);
        }
        assertTrue(detector.isUpset());
        for (int i = 0; i < 15; i++) {
            feed(detector, 2.0, 0.0, 0.0, true);
        }
        assertFalse(detector.isUpset());
        assertEquals(1.0, detector.visionStdDevScale());
        assertEquals(1, detector.events());
    }

    @Test
    void aHardStopTheWheelsAlsoFeelStillCounts() {
        CollisionDetector detector = new CollisionDetector();
        for (int i = 0; i < 10; i++) {
            feed(detector, 3.0, 0.0, 0.0, true);
        }
        feed(detector, 0.0, 15.0, 0.0, true);
        assertTrue(detector.isHit());
        assertEquals(1, detector.events());
    }

    @Test
    void aBounceRightAfterAHitIsTheSameCollision() {
        CollisionDetector detector = new CollisionDetector();
        feed(detector, 0.0, 3.0, 0.0, true);
        feed(detector, 0.0, 0.0, 0.0, true);
        feed(detector, 0.0, 3.0, 0.0, true);
        assertEquals(1, detector.events());
        for (int i = 0; i < 60; i++) {
            feed(detector, 0.0, 0.0, 0.0, true);
        }
        feed(detector, 0.0, 3.0, 0.0, true);
        assertEquals(2, detector.events());
    }

    @Test
    void aLongHitCountsOnce() {
        CollisionDetector detector = new CollisionDetector();
        for (int i = 0; i < 5; i++) {
            feed(detector, 0.0, 3.0, 0.0, true);
        }
        assertEquals(1, detector.events());
    }

    @Test
    void tiltingTrustsVisionWithoutCountingAHit() {
        CollisionDetector detector = new CollisionDetector();
        for (int i = 0; i < 10; i++) {
            feed(detector, 1.0, 2.0, Math.toRadians(15.0), true);
        }
        assertTrue(detector.isTilted());
        assertTrue(detector.isUpset());
        assertFalse(detector.isHit());
        assertEquals(0, detector.events());
    }

    @Test
    void aDisconnectedImuNeverReportsAHit() {
        CollisionDetector detector = new CollisionDetector();
        feed(detector, 0.0, 5.0, Math.toRadians(30.0), false);
        assertFalse(detector.isHit());
        assertFalse(detector.isUpset());
    }
}
