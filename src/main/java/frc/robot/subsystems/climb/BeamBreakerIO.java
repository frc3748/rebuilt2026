package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakerIO {
    @AutoLog
    class BeamBreakerInputs {
        public double distanceMeters = 0.0;
    }

    default void updateInputs(BeamBreakerInputs inputs) {}
}
