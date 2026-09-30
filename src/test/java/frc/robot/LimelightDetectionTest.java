package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.CameraConfig;
import frc.robot.subsystems.vision.CameraIO.ObjectObservation;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.CameraInputsAutoLogged;

class LimelightDetectionTest {
    @Test
    void readsEachFrameOnceAndDatesItToCapture() {
        assertTrue(HAL.initialize(500, 0));
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-test");
        CameraIOLimelight io = new CameraIOLimelight(
                new CameraConfig("Test", "limelight-test", CameraConfig.Type.LIMELIGHT_3).detector(0), false);
        CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();

        io.updateInputs(inputs);
        assertEquals(0, inputs.objectObservations.length);

        table.getEntry("tl").setDouble(80);
        table.getEntry("cl").setDouble(20);
        table.getEntry("rawdetections").setDoubleArray(new double[] {1, 5, -10, 2, 0, 0, 0, 0, 0, 0, 0, 0});
        table.getEntry("hb").setDouble(1);
        io.updateInputs(inputs);

        assertEquals(1, inputs.objectObservations.length);
        ObjectObservation object = inputs.objectObservations[0];
        assertEquals(1, object.classId());
        assertEquals(5, object.yawDegrees());
        assertEquals(-10, object.pitchDegrees());
        double arrival = table.getEntry("hb").getLastChange() / 1e6;
        assertEquals(arrival - 0.1, object.timestamp(), 1e-6);

        io.updateInputs(inputs);
        assertEquals(0, inputs.objectObservations.length);

        table.getEntry("hb").setDouble(2);
        io.updateInputs(inputs);
        assertEquals(1, inputs.objectObservations.length);
    }
}
