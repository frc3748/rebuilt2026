package frc.robot.robots.competition.autos;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.robots.competition.ActionCommands;
import frc.robot.robots.competition.CompetitionSuperstructure;

public class CustomAuto extends AutoRoutine {
    private static final int kLanes = 10;
    private static final int kStepsPerLane = 10;
    private static final Supplier<Command> kNone = Commands::none;

    private final List<LoggedDashboardChooser<Supplier<Command>>> steps = new ArrayList<>();

    public CustomAuto(CompetitionSuperstructure robot) {
        super("CUSTOM AUTO (GAME)");
        List<Method> actions = actions();
        for (int i = 0; i < kLanes * kStepsPerLane; i++) {
            LoggedDashboardChooser<Supplier<Command>> chooser = new LoggedDashboardChooser<>("Auto Parallel " + i);
            chooser.addDefaultOption("None", kNone);
            for (Method action : actions) {
                chooser.addOption(action.getName(), () -> invoke(action, robot));
            }
            steps.add(chooser);
        }
    }

    @Override
    public Command build() {
        return new DeferredCommand(() -> {
            List<Command> lanes = new ArrayList<>();
            for (int lane = 0; lane < kLanes; lane++) {
                List<Command> sequence = new ArrayList<>();
                for (int step = 0; step < kStepsPerLane; step++) {
                    Supplier<Command> selected = steps.get(lane * kStepsPerLane + step).get();
                    if (selected != null && selected != kNone) {
                        sequence.add(selected.get());
                    }
                }
                if (!sequence.isEmpty()) {
                    lanes.add(Commands.sequence(sequence.toArray(Command[]::new)));
                }
            }
            return Commands.parallel(lanes.toArray(Command[]::new));
        }, Set.of()).withName(name());
    }

    private static List<Method> actions() {
        return Arrays.stream(ActionCommands.class.getMethods())
                .filter(method -> Modifier.isStatic(method.getModifiers()))
                .filter(method -> method.getReturnType() == Command.class)
                .filter(method -> Arrays.equals(method.getParameterTypes(), new Class<?>[] {CompetitionSuperstructure.class}))
                .sorted(Comparator.comparing(Method::getName))
                .toList();
    }

    private static Command invoke(Method action, CompetitionSuperstructure robot) {
        try {
            return (Command) action.invoke(null, robot);
        } catch (ReflectiveOperationException e) {
            return Commands.print("Custom auto step failed: " + action.getName());
        }
    }
}
