package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public class DiagnosticSideways extends DiagnosticAuto {
    public DiagnosticSideways(RobotState state) {
        super(state, "Sideways", "Diagnostic: left 2 ft and back");
    }

    @Override
    protected List<Leg> legs() {
        return List.of(
                drive(0.0, 2 * kFoot, 0.0),
                drive(0.0, 0.0, 0.0));
    }
}
