package frc.robot.util;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotState;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ActionCommands;

public class CustomAutoBuilder extends AutoCommands.AutoClass {
    private final List<LoggedDashboardChooser<Supplier<Command>>> choosers = new ArrayList<>();
    private final RobotState state;
    String name;

    public CustomAutoBuilder(RobotState state) {
        this.state = state;
        this.name = "CUSTOM AUTO (GAME)";

        for (int x = 0; x < 100; x++) {
            LoggedDashboardChooser<Supplier<Command>> chooser = new LoggedDashboardChooser<>("Auto Parallel " + x);

            chooser.addDefaultOption("None", () -> new InstantCommand().withName("None"));

            for (Method method : ActionCommands.class.getDeclaredMethods()) {
                try {
                    Object result = (method.getParameterCount() == 0) ? method.invoke(null) : method.invoke(null, state);

                    if (result instanceof PathPlannerPath || result instanceof Command) {
                        chooser.addOption(method.getName(), () -> {
                            try {
                                Object freshResult = (method.getParameterCount() == 0) ? method.invoke(null) : method.invoke(null, state);
                                if (freshResult instanceof PathPlannerPath) {
                                    return AutoBuilder.followPath((PathPlannerPath) freshResult).withName(method.getName());
                                }
                                return ((Command) freshResult).withName(method.getName());
                            } catch (Exception e) {
                                return new InstantCommand().withName("Error");
                            }
                        });
                    }
                } catch (Exception e) {
                }
            }
            choosers.add(chooser);
        }
    }

    @Override
    public Command getCommand(RobotState state) {
        return new DeferredCommand(() -> {
            List<Command> parallelGroups = new ArrayList<>();
            for (int i = 0; i < 10; i++) {
                List<Command> sequentialCommands = new ArrayList<>();
                for (int j = 0; j < 10; j++) {
                    int index = (i * 10) + j;
                    Supplier<Command> selectedSupplier = choosers.get(index).get();
                    if (selectedSupplier != null) {
                        Command freshCommand = selectedSupplier.get();
                        if (!freshCommand.getName().equals("None")) {
                            sequentialCommands.add(freshCommand);
                        }
                    }
                }
                if (!sequentialCommands.isEmpty()) {
                    parallelGroups.add(new SequentialCommandGroup(sequentialCommands.toArray(new Command[0])));
                }
            }
            return new ParallelCommandGroup(parallelGroups.toArray(new Command[0]));
        }, Set.of()).withName(name);
    }

    @Override
    public List<PathPlannerPath> getAutoDisplayList(RobotState state) {
        List<PathPlannerPath> displayPaths = new ArrayList<>();
        for (int j = 0; j < 10; j++) {
            for (int i = 0; i < 10; i++) {
                int index = (i * 10) + j;
                Supplier<Command> selected = choosers.get(index).get();
                if (selected != null) {
                    Command cmd = selected.get();
                    if (!cmd.getName().equals("None")) {
                        PathPlannerPath path = findPathByName(cmd.getName());
                        if (path != null) {
                            displayPaths.add(path);
                            break;
                        }
                    }
                }
            }
        }
        return displayPaths;
    }

    private PathPlannerPath findPathByName(String name) {
        for (Method method : ActionCommands.class.getDeclaredMethods()) {
            if (method.getName().equals(name)) {
                try {
                    Object result = (method.getParameterCount() == 0) ? method.invoke(null) : method.invoke(null, state);
                    if (result instanceof PathPlannerPath) return (PathPlannerPath) result;
                } catch (Exception e) { return null; }
            }
        }
        return null;
    }
}