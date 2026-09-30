package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.TunableNumber;
import frc.robot.util.state.SubsystemManagerFactory;

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
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        StatusLogger.disableAutoLogging();
        Logger.start();

        robotState = new RobotState(Constants.kRobot.create());
        SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
    }

    @Override
    public void robotPeriodic() {
        TunableNumber.pollAll();
        CommandScheduler.getInstance().run();
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
    }
}
