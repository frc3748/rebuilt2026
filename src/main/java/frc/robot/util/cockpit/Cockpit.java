package frc.robot.util.cockpit;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class Cockpit {
    public static final int SCHEMA = 1;

    public enum Tab {
        PREMATCH,
        AUTO,
        TELEOP,
        TEST,
        TUNE,
        POSTMATCH
    }

    public enum Level {
        INFO,
        WARNING,
        ERROR
    }

    public enum Marker {
        TARGET,
        AIM,
        POINT,
        CROSSHAIR,
        ZONE
    }

    public enum Status {
        PASS,
        WARN,
        FAIL
    }

    public record Check(Status status, String detail) {
        public static Check pass(String detail) {
            return new Check(Status.PASS, detail);
        }

        public static Check warn(String detail) {
            return new Check(Status.WARN, detail);
        }

        public static Check fail(String detail) {
            return new Check(Status.FAIL, detail);
        }
    }

    private record Button(String id, String label, Tab tab, boolean confirm, Command command, BooleanSupplier on,
            LoggedNetworkBoolean pressed) {}

    private record CheckEntry(String id, String label, Supplier<Check> check) {}

    private record MarkerEntry(String id, String label, Marker kind, Supplier<Optional<Pose2d>> pose, DoubleSupplier radius) {}

    private record Assist(String id, String label, BooleanSupplier active) {}

    private record Camera(String id, String label, String stream, String fallbackUrl, BooleanSupplier connected) {}

    private record Gauge(String id, String label, String unit, DoubleSupplier value, DoubleSupplier goal, BooleanSupplier ok,
            BooleanSupplier visible) {}

    private static final List<Button> buttons = new ArrayList<>();
    private static final List<CheckEntry> checks = new ArrayList<>();
    private static final List<Gauge> gauges = new ArrayList<>();
    private static final List<Camera> cameras = new ArrayList<>();
    private static final List<MarkerEntry> markers = new ArrayList<>();
    private static final List<Assist> assists = new ArrayList<>();
    private static final int kMaxToasts = 30;
    private static final Deque<String> toasts = new ArrayDeque<>();
    private static final Map<String, Command> running = new HashMap<>();
    private static final String kBoot = Long.toString(System.currentTimeMillis(), 36);
    private static int nextToast = 1;
    private static String manifest = "";
    private static boolean ready;

    private Cockpit() {}

    public static void button(String id, String label, Tab tab, Command command) {
        add(id, label, tab, false, command, null);
    }

    public static void toggleButton(String id, String label, Tab tab, Command command, BooleanSupplier on) {
        add(id, label, tab, false, command, on);
    }

    public static void confirmButton(String id, String label, Tab tab, Command command) {
        add(id, label, tab, true, command, null);
    }

    private static void add(String id, String label, Tab tab, boolean confirm, Command command, BooleanSupplier on) {
        buttons.removeIf(button -> button.id().equals(id));
        buttons.add(new Button(id, label, tab, confirm, command, on,
                new LoggedNetworkBoolean("/Cockpit/Buttons/" + id, false)));
        manifest = "";
    }

    public static void check(String id, String label, Supplier<Check> check) {
        checks.removeIf(entry -> entry.id().equals(id));
        checks.add(new CheckEntry(id, label, check));
        manifest = "";
    }

    public static void gauge(String id, String label, String unit, DoubleSupplier value, BooleanSupplier visible) {
        gauge(id, label, unit, value, () -> Double.NaN, null, visible);
    }

    public static void gauge(String id, String label, String unit, DoubleSupplier value, DoubleSupplier goal, BooleanSupplier ok,
            BooleanSupplier visible) {
        gauges.removeIf(gauge -> gauge.id().equals(id));
        gauges.add(new Gauge(id, label, unit, value, goal, ok, visible));
    }

    public static void camera(String id, String label, String stream, String fallbackUrl, BooleanSupplier connected) {
        cameras.removeIf(camera -> camera.id().equals(id));
        cameras.add(new Camera(id, label, stream, fallbackUrl, connected));
    }

    public static void toast(Level level, String title, String detail) {
        toasts.addLast(String.join("\t", kBoot + "-" + nextToast++, level.name(), title, detail));
        while (toasts.size() > kMaxToasts) {
            toasts.removeFirst();
        }
    }

    public static void marker(String id, String label, Marker kind, Supplier<Optional<Pose2d>> pose) {
        markers.removeIf(marker -> marker.id().equals(id));
        markers.add(new MarkerEntry(id, label, kind, pose, () -> 0.0));
    }

    public static void zone(String id, String label, Supplier<Optional<Pose2d>> center, DoubleSupplier radiusMeters) {
        markers.removeIf(marker -> marker.id().equals(id));
        markers.add(new MarkerEntry(id, label, Marker.ZONE, center, radiusMeters));
    }

    public static void assist(String id, String label, BooleanSupplier active) {
        assists.removeIf(assist -> assist.id().equals(id));
        assists.add(new Assist(id, label, active));
    }

    public static void clear() {
        buttons.clear();
        checks.clear();
        gauges.clear();
        cameras.clear();
        markers.clear();
        assists.clear();
        manifest = "";
    }

    public static void update() {
        for (Button button : buttons) {
            if (button.pressed().get()) {
                button.pressed().set(false);
                CommandScheduler.getInstance().schedule(button.command());
                if (button.confirm() || button.on() != null) {
                    running.put(button.id(), button.command());
                }
            }
        }
        for (Button button : buttons) {
            Command command = running.get(button.id());
            if (command != null && !command.isScheduled()) {
                running.remove(button.id());
                toast(Level.INFO, button.on() == null ? button.label() + " done"
                        : button.label() + (button.on().getAsBoolean() ? " on" : " off"), "");
            }
        }
        Logger.recordOutput("Cockpit/Toasts", toasts.toArray(String[]::new));
        Logger.recordOutput("Cockpit/On", buttons.stream()
                .filter(button -> button.on() != null && button.on().getAsBoolean())
                .map(Button::id)
                .toArray(String[]::new));

        ready = true;
        String[] results = new String[checks.size()];
        for (int index = 0; index < checks.size(); index++) {
            CheckEntry entry = checks.get(index);
            Check result = entry.check().get();
            ready &= result.status() != Status.FAIL;
            results[index] = String.join("\t", entry.id(), result.status().name(), entry.label(), result.detail());
        }
        Logger.recordOutput("Cockpit/Checks", results);
        Logger.recordOutput("Cockpit/Ready", ready);
        Logger.recordOutput("Cockpit/Gauges", gauges.stream()
                .filter(gauge -> gauge.visible().getAsBoolean())
                .map(gauge -> String.join("\t", gauge.id(), gauge.label(), gauge.unit(),
                        Double.toString(gauge.value().getAsDouble()), Double.toString(gauge.goal().getAsDouble()),
                        gauge.ok() == null ? "" : gauge.ok().getAsBoolean() ? "1" : "0"))
                .toArray(String[]::new));
        Logger.recordOutput("Cockpit/Markers", markers.stream()
                .flatMap(marker -> marker.pose().get().stream().map(pose -> String.join("\t", marker.id(), marker.kind().name(),
                        marker.label(), Double.toString(pose.getX()), Double.toString(pose.getY()),
                        Double.toString(pose.getRotation().getDegrees()), Double.toString(marker.radius().getAsDouble()))))
                .toArray(String[]::new));
        Logger.recordOutput("Cockpit/Assists", assists.stream()
                .filter(assist -> assist.active().getAsBoolean())
                .map(Assist::label)
                .toArray(String[]::new));
        Logger.recordOutput("Cockpit/Cameras", cameras.stream()
                .map(camera -> String.join("\t", camera.id(), camera.label(), camera.stream(), camera.fallbackUrl(),
                        camera.connected().getAsBoolean() ? "1" : "0"))
                .toArray(String[]::new));

        if (manifest.isEmpty()) {
            manifest = buildManifest();
        }
        Logger.recordOutput("Cockpit/Manifest", manifest);
    }

    public static boolean isReady() {
        return ready;
    }

    private static String buildManifest() {
        StringBuilder json = new StringBuilder("{\"schema\":").append(SCHEMA).append(",\"buttons\":[");
        for (int index = 0; index < buttons.size(); index++) {
            Button button = buttons.get(index);
            if (index > 0) {
                json.append(',');
            }
            json.append("{\"id\":").append(quote(button.id()))
                    .append(",\"label\":").append(quote(button.label()))
                    .append(",\"tab\":").append(quote(button.tab().name().toLowerCase()))
                    .append(",\"confirm\":").append(button.confirm())
                    .append(",\"toggle\":").append(button.on() != null)
                    .append('}');
        }
        json.append("],\"checks\":[");
        for (int index = 0; index < checks.size(); index++) {
            if (index > 0) {
                json.append(',');
            }
            json.append("{\"id\":").append(quote(checks.get(index).id()))
                    .append(",\"label\":").append(quote(checks.get(index).label())).append('}');
        }
        return json.append("]}").toString();
    }

    private static String quote(String text) {
        return "\"" + text.replace("\\", "\\\\").replace("\"", "\\\"") + "\"";
    }
}
