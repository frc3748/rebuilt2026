package frc.robot.robots;

import java.util.List;

import frc.robot.Controls;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.util.state.StateMachine;

public interface Superstructure {
    default List<StateMachine<?>> subsystems() {
        return List.of();
    }

    default List<AutoRoutine> autos() {
        return List.of();
    }

    default void bindControls(Controls controls) {}

    default void simulationPeriodic() {}
}
