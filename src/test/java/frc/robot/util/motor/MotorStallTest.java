package frc.robot.util.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class MotorStallTest {
    private static class StuckIO implements MotorIO {
        double amps;
        String last = "";

        @Override
        public void updateInputs(MotorIOInputs inputs) {
            inputs.connected = true;
            inputs.position = 50.0;
            inputs.velocity = 0.0;
            inputs.currentAmps = amps;
        }

        @Override
        public void setPosition(double position, double feedforwardVolts, int slot) {
            last = "position";
        }

        @Override
        public void stop() {
            last = "stop";
        }
    }

    @BeforeAll
    static void boot() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }

    private static Motor motor(StuckIO io) {
        return new Motor("Motors/Stuck", io, new MotorConfig("Stuck", 90, MotorConfig.Controller.SPARK_MAX)
                .currentLimit(40)
                .softLimits(0, 100));
    }

    private static void loop(Motor motor, double seconds) {
        for (int i = 0; i < Math.round(seconds / 0.02); i++) {
            SimHooks.stepTiming(0.02);
            motor.read();
            motor.write();
        }
    }

    @Test
    void stopsPushingIntoAHardStopUntilTheGoalChanges() {
        StuckIO io = new StuckIO();
        io.amps = 38.0;
        Motor motor = motor(io);
        motor.holdPosition(80.0);
        loop(motor, 0.5);
        assertEquals("position", io.last);
        loop(motor, 1.0);
        assertEquals("stop", io.last);
        motor.holdPosition(80.0);
        loop(motor, 0.5);
        assertEquals("stop", io.last);
        motor.holdPosition(20.0);
        loop(motor, 0.1);
        assertEquals("position", io.last);
    }

    @Test
    void keepsHoldingWhenTheCurrentIsLow() {
        StuckIO io = new StuckIO();
        io.amps = 10.0;
        Motor motor = motor(io);
        motor.holdPosition(80.0);
        loop(motor, 3.0);
        assertEquals("position", io.last);
    }
}
