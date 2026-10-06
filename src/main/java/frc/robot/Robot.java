package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.BufferedReader;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryPoolMXBean;
import java.lang.management.MemoryType;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.RobotDefinition;
import frc.robot.util.LogFolder;
import frc.robot.util.TunableNumber;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.state.SubsystemManagerFactory;
import frc.robot.util.tuning.Tuning;

public class Robot extends LoggedRobot {
    private static final int kMemoryLoops = 50;
    private static final double kMegabyte = 1048576.0;
    private static final Path kMemInfo = Path.of("/proc/meminfo");
    private static final Path kProcessStatus = Path.of("/proc/self/status");
    private static final List<MemoryPoolMXBean> kPools = ManagementFactory.getMemoryPoolMXBeans();

    private final RobotState robotState;
    private int loops;

    public Robot() {
        Logger.recordMetadata("ROBOT", "2026 Recharge");
        Logger.recordMetadata("MODE", Constants.kMode.name());
        Logger.recordMetadata("ROBOT_TYPE", Constants.kRobot.name());
        Logger.recordMetadata("GIT_SHA", BuildInfo.GIT_SHA);
        Logger.recordMetadata("GIT_BRANCH", BuildInfo.GIT_BRANCH);
        Logger.recordMetadata("GIT_DIRTY", Boolean.toString(BuildInfo.GIT_DIRTY));
        Logger.recordMetadata("BUILD_DATE", BuildInfo.BUILD_DATE);
        String logFolder = LogFolder.choose();
        Logger.recordMetadata("LOG_FOLDER", logFolder);
        Logger.addDataReceiver(new WPILOGWriter(logFolder));
        Logger.addDataReceiver(new NT4Publisher());
        StatusLogger.disableAutoLogging();
        SignalLogger.enableAutoLogging(false);
        Logger.start();

        RobotDefinition definition = Constants.kRobot.create();
        Tuning.boot(Constants.kRobot.name(), definition.getClass());
        definition.tune();
        robotState = new RobotState(definition);
        SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
    }

    @Override
    public void robotPeriodic() {
        if (loops++ % kMemoryLoops == 0) {
            logMemory();
        }
        TunableNumber.pollAll();
        CommandScheduler.getInstance().run();
        Cockpit.update();
    }

    @Override
    public void simulationPeriodic() {
        robotState.updateSimulation();
    }

    @Override
    public void disabledInit() {
        SubsystemManagerFactory.getInstance().disableAllSubsystems();
    }

    private static void logMemory() {
        Runtime runtime = Runtime.getRuntime();
        Logger.recordOutput("JVM/HeapUsedMB", (runtime.totalMemory() - runtime.freeMemory()) / kMegabyte);
        Logger.recordOutput("JVM/HeapTotalMB", runtime.totalMemory() / kMegabyte);
        Logger.recordOutput("JVM/HeapMaxMB", runtime.maxMemory() / kMegabyte);
        double nonHeap = 0.0;
        double code = 0.0;
        double metaspace = 0.0;
        for (MemoryPoolMXBean pool : kPools) {
            if (pool.getType() != MemoryType.NON_HEAP) {
                continue;
            }
            double committed = pool.getUsage().getCommitted() / kMegabyte;
            nonHeap += committed;
            if (pool.getName().startsWith("Code")) {
                code += committed;
            } else if (pool.getName().equals("Metaspace")) {
                metaspace = committed;
            }
        }
        Logger.recordOutput("JVM/NonHeapMB", nonHeap);
        Logger.recordOutput("JVM/CodeCacheMB", code);
        Logger.recordOutput("JVM/MetaspaceMB", metaspace);
        if (Constants.kMode == Constants.Mode.REAL) {
            Logger.recordOutput("System/MemAvailableMB", procValue(kMemInfo, "MemAvailable:") / 1024.0);
            Logger.recordOutput("System/MemFreeMB", procValue(kMemInfo, "MemFree:") / 1024.0);
            Logger.recordOutput("JVM/ProcessMB", procValue(kProcessStatus, "VmRSS:") / 1024.0);
            Logger.recordOutput("JVM/ProcessPeakMB", procValue(kProcessStatus, "VmHWM:") / 1024.0);
            Logger.recordOutput("JVM/Threads", procValue(kProcessStatus, "Threads:"));
        }
    }

    private static double procValue(Path file, String field) {
        try (BufferedReader reader = Files.newBufferedReader(file)) {
            for (String line = reader.readLine(); line != null; line = reader.readLine()) {
                if (line.startsWith(field)) {
                    return Long.parseLong(line.replaceAll("\\D", ""));
                }
            }
        } catch (IOException | NumberFormatException e) {
            return Double.NaN;
        }
        return Double.NaN;
    }

    @Override
    public void autonomousInit() {
        SubsystemManagerFactory.getInstance().notifyAutonomousStart();
    }

    @Override
    public void teleopInit() {
        SubsystemManagerFactory.getInstance().notifyTeleopStart();
    }

    @Override
    public void testInit() {
        SubsystemManagerFactory.getInstance().notifyTestStart();
        CommandScheduler.getInstance().cancelAll();
    }
}
