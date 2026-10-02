package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public class DiagnosticTurnWhileDriving extends DiagnosticAuto {
    public DiagnosticTurnWhileDriving(RobotState state) {
        super(state, "TurnWhileDriving", "Diagnostic: 2 m turning 180 and back");
    }

    @Override
    protected List<Leg> legs() {
        return List.of(
                drive(2.0, 0.0, 180.0),
                drive(0.0, 0.0, 0.0));
    }
}
