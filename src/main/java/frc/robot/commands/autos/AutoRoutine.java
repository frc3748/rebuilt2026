package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class AutoRoutine {
    private final String name;
    private final String[] pathNames;
    private String mode = "";

    protected AutoRoutine(String name, String... pathNames) {
        this.name = name;
        this.pathNames = pathNames;
    }

    public String name() {
        return name;
    }

    public String mode() {
        return mode;
    }

    public AutoRoutine mode(String mode) {
        this.mode = mode;
        return this;
    }

    public abstract Command build();

    public List<PathPlannerPath> previewPaths() {
        try {
            return new ArrayList<>(loadPaths().values());
        } catch (Exception e) {
            return List.of();
        }
    }

    protected final String firstPathName() {
        return pathNames[0];
    }

    protected final Map<String, PathPlannerPath> loadPaths() throws Exception {
        Map<String, PathPlannerPath> paths = new LinkedHashMap<>();
        for (String pathName : pathNames) {
            paths.put(pathName, PathPlannerPath.fromPathFile(pathName));
        }
        return paths;
    }

    public static AutoRoutine none() {
        return new AutoRoutine("None") {
            @Override
            public Command build() {
                return Commands.none().withName(name());
            }
        };
    }

    public static AutoRoutine pathPlanner(String autoName) {
        return new AutoRoutine(autoName) {
            @Override
            public Command build() {
                return new PathPlannerAuto(autoName);
            }

            @Override
            public List<PathPlannerPath> previewPaths() {
                try {
                    return PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                } catch (Exception e) {
                    return List.of();
                }
            }
        };
    }
}
