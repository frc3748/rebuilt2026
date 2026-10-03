package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public final class Autos {
    public static List<AutoRoutine> all(RobotState state) {
        return List.of(
                new CenterOnlyStarting8(state).mode("Starting 8"),
                new DepotOnlyStarting8(state).mode("Starting 8"),
                new DepotSideToDepot(state).mode("To station"),
                new DepotSideToDepotEndAtMid(state).mode("To station, end mid"),
                new HpOnlyStarting8(state).mode("Starting 8"),
                new HpSideToHp(state).mode("To station"),
                new HpSideToHpEndAtMid(state).mode("To station, end mid"),
                new DepotSideDepotMidHalfSweep(state).mode("Half sweep"),
                new DepotSideQuickShoot(state).mode("Quick shoot"),
                new HpSideQuickShoot(state).mode("Quick shoot"),
                new DepotSideCircuitShoot(state).mode("Circuit"),
                new DepotSideBlair(state).mode("Blair"),
                new HpSideBlair(state).mode("Blair"),
                new DepotSideBump(state).mode("Bump"),
                new DiagnosticForwardBack(state),
                new DiagnosticSideways(state),
                new DiagnosticSquare(state),
                new DiagnosticSpin(state),
                new DiagnosticTurnWhileDriving(state),
                new DiagnosticAutoSpeed(state),
                new MeasureWheelRadius(state),
                new MeasureFeedforward(state),
                new MeasureSlipCurrent(state),
                new MeasureSteering(state));
    }

    private Autos() {}
}
