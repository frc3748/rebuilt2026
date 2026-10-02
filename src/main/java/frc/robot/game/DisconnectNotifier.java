package frc.robot.game;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Level;

public class DisconnectNotifier {
    private static final double kGraceSeconds = 2.0;
    private static final int kLoopsPerCheck = 5;

    private final Supplier<List<String>> devices;
    private final Set<String> gyros;
    private Set<String> missing = new LinkedHashSet<>();
    private int loops;
    private int events;

    public DisconnectNotifier(Supplier<List<String>> devices, Set<String> gyros) {
        this.devices = devices;
        this.gyros = gyros;
    }

    public void update() {
        if (Timer.getFPGATimestamp() < kGraceSeconds || ++loops % kLoopsPerCheck != 0) {
            return;
        }
        Set<String> now = new LinkedHashSet<>(devices.get());
        List<String> lost = new ArrayList<>(now);
        lost.removeAll(missing);
        List<String> back = new ArrayList<>(missing);
        back.removeAll(now);
        if (!lost.isEmpty()) {
            events += lost.size();
            Cockpit.toast(Level.ERROR, String.join(", ", lost) + " disconnected", detail(lost));
        }
        if (!back.isEmpty()) {
            Cockpit.toast(Level.INFO, String.join(", ", back) + " connected again", "");
        }
        missing = now;
        Logger.recordOutput("Disconnects/Devices", missing.toArray(String[]::new));
        Logger.recordOutput("Disconnects/Events", events);
    }

    private String detail(List<String> lost) {
        if (lost.stream().anyMatch(gyros::contains)) {
            return "Heading now comes from the wheels, so field-relative driving and heading lock will drift. Check its cable.";
        }
        return "Check its CAN wiring and breaker";
    }
}
