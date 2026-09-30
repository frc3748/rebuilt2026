package frc.robot.subsystems.climb;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class BeamBreakerTOF implements BeamBreakerIO {
    private final TimeOfFlight sensor;

    public BeamBreakerTOF(int canId) {
        sensor = new TimeOfFlight(canId);
        sensor.setRangingMode(RangingMode.Medium, 24);
    }

    @Override
    public void updateInputs(BeamBreakerInputs inputs) {
        inputs.distanceMeters = sensor.getRange() / 1000.0;
    }
}
