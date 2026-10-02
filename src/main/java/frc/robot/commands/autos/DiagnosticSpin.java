package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public class DiagnosticSpin extends DiagnosticAuto {
    public DiagnosticSpin(RobotState state) {
        super(state, "Spin", "Diagnostic: full spin");
    }

    @Override
    protected List<Leg> legs() {
        return List.of(
                turn(90.0),
                turn(180.0),
                turn(270.0),
                turn(360.0));
    }
}
