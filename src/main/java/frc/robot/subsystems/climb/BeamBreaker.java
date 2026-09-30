package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.state.Hardware;

public class BeamBreaker implements Hardware {
    private final String name;
    private final BeamBreakerIO io;
    private final BeamBreakerInputsAutoLogged inputs = new BeamBreakerInputsAutoLogged();

    public BeamBreaker(String name, int canId) {
        this.name = name;
        io = Constants.kMode == Mode.REAL ? new BeamBreakerTOF(canId) : new BeamBreakerIO() {};
    }

    @Override
    public void read() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public double getDistanceMeters() {
        return inputs.distanceMeters;
    }
}
