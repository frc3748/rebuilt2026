package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Level;
import frc.robot.util.tuning.Tuning;

public abstract class MeasureAuto extends AutoRoutine {
    private static final Map<String, String> results = new LinkedHashMap<>();

    public enum Outcome {
        SAME,
        CHANGED,
        FAILED
    }

    public record Result(Outcome outcome, String summary, Map<String, Double> measured, Map<String, Double> proposed,
            List<String> applied) {}

    protected final RobotState state;
    private final String id;
    private Result lastResult;

    protected MeasureAuto(RobotState state, String id, String name) {
        super(name);
        this.state = state;
        this.id = id;
        mode("Diagnostic");
    }

    protected abstract Command measure();

    public static String[] results() {
        return results.values().toArray(String[]::new);
    }

    public Result lastResult() {
        return lastResult;
    }

    @Override
    public final Command build() {
        Drive drive = state.getDrive();
        return measure().finallyDo(drive::stop).withName(name());
    }

    protected final void report(Outcome outcome, String summary, Map<String, Double> measured, Map<String, Double> proposed) {
        measured.forEach((metric, value) -> Logger.recordOutput("Measure/" + id + "/" + metric, value));
        List<String> applied = new ArrayList<>();
        if (outcome == Outcome.CHANGED) {
            proposed.forEach((key, value) -> {
                if (Tuning.propose(key, value)) {
                    applied.add(key);
                }
            });
        }
        lastResult = new Result(outcome, summary, measured, proposed, applied);
        String next = outcome != Outcome.CHANGED || proposed.isEmpty() ? ""
                : !applied.isEmpty() ? "Put on the Tune tab. Press Save to keep it."
                        : !Tuning.isActive() ? "Turn on tuning mode and run it again to put it on the Tune tab."
                                : "This robot can't tune it live, so put it in the code by hand.";
        Logger.recordOutput("Measure/" + id + "/Outcome", outcome.name());
        Logger.recordOutput("Measure/" + id + "/Summary", summary);
        results.put(id, String.join("\t", name(), outcome.name().toLowerCase(), summary, next));
        Level level = outcome == Outcome.FAILED ? Level.WARNING : Level.INFO;
        String title = name() + switch (outcome) {
            case SAME -> " matches the code";
            case CHANGED -> " found a better value";
            case FAILED -> " couldn't measure";
        };
        Cockpit.toast(level, title, next.isEmpty() ? summary : summary + ". " + next);
    }

    protected final void fail(String reason) {
        report(Outcome.FAILED, reason, Map.of(), Map.of());
    }
}
