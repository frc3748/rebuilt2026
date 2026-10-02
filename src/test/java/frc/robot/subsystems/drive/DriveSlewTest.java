package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DriveSlewTest {
    private static final double kDt = 0.02;

    @Test
    void speedsUpAtTheAccelerationLimit() {
        DriveConfig config = new DriveConfig();
        DriveSlew slew = new DriveSlew(config);
        ChassisSpeeds out = slew.limit(new ChassisSpeeds(4.0, 0.0, 0.0), true);
        assertEquals(config.teleopAcceleration * kDt, out.vxMetersPerSecond, 1e-9);
        for (int i = 0; i < 100; i++) {
            out = slew.limit(new ChassisSpeeds(4.0, 0.0, 0.0), true);
        }
        assertEquals(4.0, out.vxMetersPerSecond, 1e-9);
    }

    @Test
    void slowsDownRightAway() {
        DriveSlew slew = new DriveSlew(new DriveConfig());
        slew.reset(new ChassisSpeeds(4.0, 0.0, 0.0));
        assertEquals(0.0, slew.limit(new ChassisSpeeds(), true).vxMetersPerSecond, 1e-9);
        assertEquals(new DriveConfig().teleopAcceleration * kDt, slew.limit(new ChassisSpeeds(4.0, 0.0, 0.0), true).vxMetersPerSecond, 1e-9);
    }

    @Test
    void changesDirectionRightAwayAtTheSameSpeed() {
        DriveSlew slew = new DriveSlew(new DriveConfig());
        slew.reset(new ChassisSpeeds(4.0, 0.0, 0.0));
        ChassisSpeeds out = slew.limit(new ChassisSpeeds(0.0, 4.0, 0.0), true);
        assertEquals(0.0, out.vxMetersPerSecond, 1e-9);
        assertEquals(4.0, out.vyMetersPerSecond, 1e-9);
        out = slew.limit(new ChassisSpeeds(-4.0, 0.0, 0.0), true);
        assertEquals(-4.0, out.vxMetersPerSecond, 1e-9);
    }

    @Test
    void keepsTheDirectionWhileSpeedingUp() {
        DriveSlew slew = new DriveSlew(new DriveConfig());
        ChassisSpeeds out = slew.limit(new ChassisSpeeds(3.0, 4.0, 0.0), true);
        assertEquals(4.0 / 3.0, out.vyMetersPerSecond / out.vxMetersPerSecond, 1e-9);
    }

    @Test
    void onlyLimitsSpinningUpWhenAsked() {
        DriveConfig config = new DriveConfig();
        DriveSlew slew = new DriveSlew(config);
        assertEquals(-config.teleopTurnAcceleration * kDt, slew.limit(new ChassisSpeeds(0.0, 0.0, -6.0), true).omegaRadiansPerSecond, 1e-9);
        assertEquals(6.0, slew.limit(new ChassisSpeeds(0.0, 0.0, 6.0), false).omegaRadiansPerSecond, 1e-9);
        assertEquals(-6.0, slew.limit(new ChassisSpeeds(0.0, 0.0, -6.0), true).omegaRadiansPerSecond, 1e-9);
    }
}
