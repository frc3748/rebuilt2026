package frc.robot.util;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

public class BatteryTracker {
    public static final String kUnknown = "Unknown";

    private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Battery");
    private final Alert missing = new Alert("Pick the battery on the dashboard", AlertType.kWarning);

    public BatteryTracker() {
        chooser.addDefaultOption(kUnknown, kUnknown);
        for (String id : loadBatteries()) {
            chooser.addOption(id, id);
        }
        Logger.recordOutput("Battery/BootVoltage", RobotController.getBatteryVoltage());
    }

    public void update() {
        String id = getId();
        Logger.recordOutput("Battery/Id", id);
        missing.set(kUnknown.equals(id) && DriverStation.isEnabled());
    }

    public String getId() {
        String id = chooser.get();
        return id == null ? kUnknown : id;
    }

    private static List<String> loadBatteries() {
        List<String> ids = new ArrayList<>();
        try {
            JsonNode root = new ObjectMapper().readTree(new File(Filesystem.getDeployDirectory(), "batteries.json"));
            root.path("batteries").forEach(battery -> ids.add(battery.path("id").asText(battery.asText())));
        } catch (IOException e) {
            DriverStation.reportWarning("Couldn't read batteries.json: " + e.getMessage(), false);
        }
        return ids;
    }
}
