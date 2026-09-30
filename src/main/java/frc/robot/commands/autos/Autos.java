package frc.robot.commands.autos;

import java.util.List;

import frc.robot.RobotState;

public final class Autos {
    public static List<AutoRoutine> all(RobotState state) {
        return List.of(
                new CenterOnlyStarting8(state),
                new DepotOnlyStarting8(state),
                new DepotSideToDepot(state),
                new DepotSideToDepotEndAtMid(state),
                new HpOnlyStarting8(state),
                new HpSideToHp(state),
                new HpSideToHpEndAtMid(state),
                new DepotSideDepotMidHalfSweep(state),
                new DepotSideQuickShoot(state),
                new HpSideQuickShoot(state),
                new DepotSideCircuitShoot(state),
                new DepotSideBlair(state),
                new HpSideBlair(state),
                new DepotSideBump(state),
                new CustomAuto(state));
    }

    private Autos() {}
}
