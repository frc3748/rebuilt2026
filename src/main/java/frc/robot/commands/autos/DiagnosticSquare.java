package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public class DiagnosticSquare extends DiagnosticAuto {
    public DiagnosticSquare(RobotState state) {
        super(state, "Square", "Diagnostic: 1 m square");
    }

    @Override
    protected List<Leg> legs() {
        return List.of(
                drive(1.0, 0.0, 0.0),
                drive(1.0, 1.0, 0.0),
                drive(0.0, 1.0, 0.0),
                drive(0.0, 0.0, 0.0));
    }
}
