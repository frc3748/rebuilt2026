package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

class SlipCorrectorTest {
    private static final Translation2d[] kModules = {
        new Translation2d(0.35, 0.35), new Translation2d(0.35, -0.35), new Translation2d(-0.35, 0.35), new Translation2d(-0.35, -0.35)};
    private static final double kDt = 0.01;

    private static SwerveModulePosition[] forward(double... meters) {
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            deltas[i] = new SwerveModulePosition(meters[i], Rotation2d.kZero);
        }
        return deltas;
    }

    @Test
    void cleanDrivingPassesThroughUntouched() {
        SlipCorrector slip = new SlipCorrector(kModules);
        SwerveModulePosition[] deltas = forward(0.02, 0.02, 0.02, 0.02);
        SwerveModulePosition[] corrected = slip.correct(deltas, 0.0, kDt);
        for (int i = 0; i < 4; i++) {
            assertSame(deltas[i], corrected[i]);
        }
        assertArrayEquals(new boolean[4], slip.slippingModules());
        assertEquals(0, slip.events());
    }

    @Test
    void aSpinningWheelIsReplacedByWhatTheOthersAgreeOn() {
        SlipCorrector slip = new SlipCorrector(kModules);
        SwerveModulePosition[] corrected = slip.correct(forward(0.02, 0.02, 0.02, 0.05), 0.0, kDt);
        assertTrue(slip.slippingModules()[3]);
        assertFalse(slip.slippingModules()[0]);
        assertEquals(0.02, corrected[3].distanceMeters, 1e-9);
        assertEquals(1, slip.events());
    }

    @Test
    void turningInPlaceIsNotSlip() {
        SlipCorrector slip = new SlipCorrector(kModules);
        double dtheta = 0.05;
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            Translation2d arc = new Translation2d(-dtheta * kModules[i].getY(), dtheta * kModules[i].getX());
            deltas[i] = new SwerveModulePosition(arc.getNorm(), arc.getAngle());
        }
        slip.correct(deltas, dtheta, kDt);
        assertArrayEquals(new boolean[4], slip.slippingModules());
    }

    @Test
    void wheelsSpeedingUpFasterThanTheRobotScaleOdometryDown() {
        SlipCorrector slip = new SlipCorrector(kModules);
        slip.update(new ChassisSpeeds(1.0, 0.0, 0.0), true, 0.0, 0.0, 0.0, 0.0, 0.02);
        slip.update(new ChassisSpeeds(1.3, 0.0, 0.0), true, 0.1, 0.0, 0.0, 0.0, 0.02);
        assertTrue(slip.isRobotSlipping());
        assertTrue(slip.scale() < 1.0);
        SwerveModulePosition[] corrected = slip.correct(forward(0.026, 0.026, 0.026, 0.026), 0.0, kDt);
        assertTrue(corrected[0].distanceMeters < 0.026);
    }

    @Test
    void theImuIsIgnoredOnABump() {
        SlipCorrector slip = new SlipCorrector(kModules);
        slip.update(new ChassisSpeeds(1.0, 0.0, 0.0), true, 0.0, 0.0, 0.0, 0.0, 0.02);
        slip.update(new ChassisSpeeds(1.3, 0.0, 0.0), true, 0.1, 0.0, Math.toRadians(15.0), 0.0, 0.02);
        assertFalse(slip.isRobotSlipping());
        assertEquals(1.0, slip.scale());
    }
}
