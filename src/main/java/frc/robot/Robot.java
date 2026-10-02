package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SelfTest;
import frc.robot.robots.RobotDefinition;
import frc.robot.util.LogFolder;
import frc.robot.util.TunableNumber;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.state.SubsystemManagerFactory;
import frc.robot.util.tuning.Tuning;

public class Robot extends LoggedRobot {
    private final RobotState robotState;

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
        Logger.start();

        RobotDefinition definition = Constants.kRobot.create();
        Tuning.boot(Constants.kRobot.name(), definition.getClass());
        definition.tune();
        robotState = new RobotState(definition);
        SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
    }

    @Override
    public void robotPeriodic() {
        Runtime runtime = Runtime.getRuntime();
        Logger.recordOutput("JVM/HeapUsedMB", (runtime.totalMemory() - runtime.freeMemory()) / 1048576.0);
        Logger.recordOutput("JVM/HeapTotalMB", runtime.totalMemory() / 1048576.0);
        Logger.recordOutput("JVM/HeapMaxMB", runtime.maxMemory() / 1048576.0);
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
        CommandScheduler.getInstance().schedule(SelfTest.build(robotState));
    }
}
