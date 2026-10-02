package frc.robot.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.wpilibj.RobotBase;

public final class LogFolder {
    private static final String kUsbLogs = "/U/logs";
    private static final File kInternalLogs = new File("/home/lvuser/logs");
    private static final long kInternalBudgetBytes = 150L * 1024 * 1024;

    private static boolean usb;

    private LogFolder() {}

    public static String choose() {
        if (RobotBase.isSimulation()) {
            return new File("logs").getAbsolutePath();
        }
        if (usbMounted()) {
            File folder = new File(kUsbLogs);
            if (folder.isDirectory() || folder.mkdirs()) {
                usb = true;
                return kUsbLogs;
            }
        }
        kInternalLogs.mkdirs();
        trim(kInternalLogs, kInternalBudgetBytes);
        return kInternalLogs.getPath();
    }

    public static boolean isUsb() {
        return usb || RobotBase.isSimulation();
    }

    static boolean usbMounted() {
        try {
            return Files.readAllLines(Path.of("/proc/mounts")).stream()
                    .map(line -> line.split(" "))
                    .anyMatch(fields -> fields.length > 1 && (fields[1].startsWith("/media/sd") || fields[1].equalsIgnoreCase("/u")));
        } catch (IOException e) {
            return new File("/U").isDirectory();
        }
    }

    static void trim(File folder, long budgetBytes) {
        File[] logs = folder.listFiles((dir, name) -> name.endsWith(".wpilog"));
        if (logs == null) {
            return;
        }
        Arrays.sort(logs, Comparator.comparingLong(File::lastModified));
        long total = Arrays.stream(logs).mapToLong(File::length).sum();
        for (File log : logs) {
            if (total <= budgetBytes) {
                return;
            }
            long size = log.length();
            if (log.delete()) {
                total -= size;
            }
        }
    }
}
