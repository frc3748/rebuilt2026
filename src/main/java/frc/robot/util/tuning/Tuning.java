package frc.robot.util.tuning;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public final class Tuning {
    private static final double kEpsilon = 1e-9;
    private static final File kRealStore = new File("/home/lvuser/tuning.json");
    private static final File kSimStore = new File("build/tuning-sim.json");
    private static final ObjectMapper kJson = new ObjectMapper();

    private static final Map<String, Value> values = new LinkedHashMap<>();
    private static final Map<String, Overridden> overrides = new LinkedHashMap<>();
    private static final Map<String, ObjectNode> robots = new LinkedHashMap<>();
    private static final Alert storeAlert = new Alert("", AlertType.kWarning);

    private static ObjectNode stored;
    private static String robot = "";
    private static boolean enabled;
    private static boolean storeDirty;
    private static boolean catalogDirty = true;
    private static String[] catalog = new String[0];

    private record Overridden(double value, Source source) {}

    public static final class Value {
        private final String key;
        private final double declared;
        private final Source source;
        private final LoggedNetworkNumber live;
        private Overridden override;
        private Double saved;
        private boolean integer;
        private boolean restart;
        private String unit = "";
        private double scale = 1.0;
        private boolean defaultLogged;

        private Value(String key, double declared, Source source) {
            this.key = key;
            this.declared = declared;
            this.source = source;
            override = overrides.get(key);
            loadSaved();
            live = new LoggedNetworkNumber("/Tunable/" + key, baseline());
        }

        public double get() {
            double value = isActive() ? live.get() : baseline();
            return integer ? Math.round(value) : value;
        }

        public void integer() {
            integer = true;
            catalogDirty = true;
        }

        public void restartToApply() {
            restart = true;
            catalogDirty = true;
        }

        public void display(String unit, double scale) {
            this.unit = unit;
            this.scale = scale;
            catalogDirty = true;
        }

        double code() {
            return override != null ? override.value() : declared;
        }

        double baseline() {
            return saved != null ? saved : code();
        }

        private double liveValue() {
            double value = live.get();
            return integer ? Math.round(value) : value;
        }

        private void loadSaved() {
            JsonNode record = stored == null ? null : stored.get(key);
            if (record == null) {
                return;
            }
            double value = record.path("value").asDouble();
            if (same(value, code())) {
                stored.remove(key);
                saved = null;
                storeDirty = true;
            } else {
                saved = value;
            }
        }

        private ObjectNode record() {
            ObjectNode record = kJson.createObjectNode();
            record.put("value", saved);
            record.put("code", code());
            record.put("declared", declared);
            record.put("integer", integer);
            record.set("source", kJson.valueToTree(source.toJson()));
            if (override != null) {
                ObjectNode overrideNode = record.putObject("override");
                overrideNode.put("file", override.source().files().isEmpty() ? "" : override.source().files().get(0));
                overrideNode.put("line", override.source().line());
            }
            return record;
        }

        private String line() {
            String flags = (integer ? "i" : "") + (restart ? "r" : "") + (override != null ? "o" : "");
            String where = override != null ? override.source().describe() : source.describe();
            return String.join("\t", key, Double.toString(code()), saved == null ? "" : Double.toString(saved), where, flags,
                    unit, Double.toString(scale));
        }
    }

    private Tuning() {}

    public static void boot(String robotName, Class<?> robotClass) {
        robot = robotName;
        if (Constants.kMode == Constants.Mode.REPLAY) {
            return;
        }
        File store = store();
        if (store.exists()) {
            try {
                JsonNode root = kJson.readTree(store);
                root.path("robots").fields().forEachRemaining(entry -> {
                    if (entry.getValue() instanceof ObjectNode node) {
                        robots.put(entry.getKey(), node);
                    }
                });
            } catch (IOException e) {
                storeAlert.setText("Couldn't read saved tuning: " + e.getMessage());
                storeAlert.set(true);
            }
        }
        ObjectNode mine = robots.computeIfAbsent(robot, name -> kJson.createObjectNode());
        mine.put("robotFile", Source.file(robotClass));
        stored = mine.get("values") instanceof ObjectNode node ? node : mine.putObject("values");
        values.values().forEach(value -> {
            value.loadSaved();
            value.live.set(value.baseline());
        });
        catalogDirty = true;
    }

    public static Value value(String key, double declared, Source source) {
        Value existing = values.get(key);
        if (existing != null) {
            return existing;
        }
        Value created = new Value(key, declared, source);
        values.put(key, created);
        catalogDirty = true;
        return created;
    }

    public static void override(String key, double value) {
        Overridden overridden = new Overridden(value, Source.caller(Tuning.class, "override", "override", 1));
        overrides.put(key, overridden);
        Value existing = values.get(key);
        if (existing != null) {
            existing.override = overridden;
            existing.loadSaved();
            existing.live.set(existing.baseline());
            catalogDirty = true;
        }
    }

    public static boolean isActive() {
        return enabled && !DriverStation.isFMSAttached();
    }

    public static boolean isEnabled() {
        return enabled;
    }

    public static void setEnabled(boolean on) {
        enabled = on;
    }

    public static int unsaved() {
        if (!isActive()) {
            return 0;
        }
        int count = 0;
        for (Value value : values.values()) {
            if (!same(value.liveValue(), value.baseline())) {
                count++;
            }
        }
        return count;
    }

    public static int pending() {
        return stored == null ? 0 : stored.size();
    }

    public static int save() {
        if (!isActive()) {
            return 0;
        }
        int changed = 0;
        for (Value value : values.values()) {
            double live = value.liveValue();
            if (same(live, value.baseline())) {
                continue;
            }
            value.saved = same(live, value.code()) ? null : live;
            if (stored != null) {
                if (value.saved == null) {
                    stored.remove(value.key);
                } else {
                    stored.set(value.key, value.record());
                }
            }
            changed++;
        }
        if (changed > 0) {
            catalogDirty = true;
            write();
        }
        return changed;
    }

    public static int revert() {
        int reverted = 0;
        for (Value value : values.values()) {
            if (!same(value.liveValue(), value.baseline())) {
                value.live.set(value.baseline());
                reverted++;
            }
        }
        return reverted;
    }

    public static int forget() {
        int forgotten = pending();
        for (Value value : values.values()) {
            if (value.saved != null) {
                value.saved = null;
                value.live.set(value.code());
            }
        }
        if (stored != null) {
            stored.removeAll();
        }
        catalogDirty = true;
        write();
        return forgotten;
    }

    public static void periodic() {
        if (catalogDirty) {
            List<String> lines = new ArrayList<>();
            values.values().forEach(value -> lines.add(value.line()));
            catalog = lines.toArray(String[]::new);
            catalogDirty = false;
        }
        for (Value value : values.values()) {
            if (!value.defaultLogged) {
                Logger.recordOutput("TunableDefaults/" + value.key, value.code());
                value.defaultLogged = true;
            }
        }
        Logger.recordOutput("Tuning/Catalog", catalog);
        Logger.recordOutput("Tuning/Enabled", enabled);
        Logger.recordOutput("Tuning/Active", isActive());
        Logger.recordOutput("Tuning/Pending", pending());
        Logger.recordOutput("Tuning/Robot", robot);
        if (storeDirty) {
            write();
        }
    }

    private static void write() {
        storeDirty = false;
        if (Constants.kMode == Constants.Mode.REPLAY) {
            return;
        }
        ObjectNode root = kJson.createObjectNode();
        ObjectNode robotsNode = root.putObject("robots");
        robots.forEach((name, node) -> {
            if (node.path("values").size() > 0) {
                robotsNode.set(name, node);
            }
        });
        File store = store();
        try {
            if (robotsNode.isEmpty()) {
                Files.deleteIfExists(store.toPath());
                return;
            }
            File parent = store.getAbsoluteFile().getParentFile();
            if (parent != null) {
                parent.mkdirs();
            }
            File temp = new File(store.getPath() + ".tmp");
            kJson.writerWithDefaultPrettyPrinter().writeValue(temp, root);
            Files.move(temp.toPath(), store.toPath(), StandardCopyOption.REPLACE_EXISTING);
            storeAlert.set(false);
        } catch (IOException e) {
            storeAlert.setText("Couldn't save tuning: " + e.getMessage());
            storeAlert.set(true);
        }
    }

    private static File store() {
        return RobotBase.isReal() ? kRealStore : kSimStore;
    }

    private static boolean same(double a, double b) {
        return Math.abs(a - b) <= kEpsilon * Math.max(1.0, Math.abs(b));
    }
}
