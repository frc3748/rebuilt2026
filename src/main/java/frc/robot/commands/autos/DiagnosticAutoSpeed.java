package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public class DiagnosticAutoSpeed extends DiagnosticAuto {
    public DiagnosticAutoSpeed(RobotState state) {
        super(state, "AutoSpeed", "Diagnostic: 3 m at auto speed and back");
    }

    @Override
    protected List<Leg> legs() {
        return List.of(
                atAutoSpeed(3.0, 0.0, 0.0),
                atAutoSpeed(0.0, 0.0, 0.0));
    }
}
