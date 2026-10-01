package frc.robot.util.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.robots.secondary.SecondaryDrive;
import frc.robot.util.TunableNumber;
import frc.robot.util.motor.MotorConfig.Controller;
import frc.robot.util.tuning.Source;

class TuningSourceTest {
    private static final String kFile = "src/main/java/frc/robot/util/motor/TuningSourceTest.java";

    @Test
    void builderCallsRecordTheirLine() {
        MotorConfig config = new MotorConfig("Test Motor", 1, Controller.SPARK_MAX)
                .currentLimit(30)
                .pid(0.1, 0.0, 0.2)
                .feedforward(0.3, 0.4, 0.0)
                .maxMotion(10, 20, 0.5);
        Source kP = config.sources.get("kP");
        assertEquals(List.of(kFile), kP.files());
        assertEquals(23, kP.line());
        assertEquals("pid", kP.call());
        assertEquals(0, kP.argument());
        assertEquals(2, config.sources.get("kD").argument());
        assertEquals(24, config.sources.get("kV").line());
        assertEquals(1, config.sources.get("kV").argument());
        assertEquals(22, config.sources.get("Current Limit").line());
        assertEquals(25, config.sources.get("kCruiseVel").line());
        assertTrue(!config.sources.containsKey("kG"));
    }

    @Test
    void tunableNumbersRecordTheirCaller() {
        TunableNumber tunable = new TunableNumber("Test/Caller", 1.5);
        assertEquals(1.5, tunable.get());
        Source source = Source.caller(TuningSourceTest.class, "tunableNumbersRecordTheirCaller", "x", 0);
        assertTrue(source.files().isEmpty() || source.line() > 0);
    }

    @Test
    void fieldsListTheClassChainUpToTheDeclaringClass() {
        DriveConfig config = new SecondaryDrive();
        Source source = Source.field(config, "driveKp");
        assertEquals(List.of(
                "src/main/java/frc/robot/robots/secondary/SecondaryDrive.java",
                "src/main/java/frc/robot/robots/comp/CompDrive.java",
                "src/main/java/frc/robot/subsystems/drive/DriveConfig.java"), source.files());
        assertEquals("driveKp", source.field());
        assertEquals(config.driveKp, Source.read(config, "driveKp"));
    }
}
