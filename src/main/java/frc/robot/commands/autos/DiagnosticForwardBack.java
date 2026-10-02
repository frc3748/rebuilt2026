package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public class DiagnosticForwardBack extends DiagnosticAuto {
    public DiagnosticForwardBack(RobotState state) {
        super(state, "ForwardBack", "Diagnostic: forward 2 ft and back");
    }

    @Override
    protected List<Leg> legs() {
        return List.of(
                drive(2 * kFoot, 0.0, 0.0),
                drive(0.0, 0.0, 0.0));
    }
}
