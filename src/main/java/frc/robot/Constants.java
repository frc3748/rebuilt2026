package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.robots.RobotType;

public final class Constants {
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    private static final Mode kSimMode = Mode.SIM;
    public static final Mode kMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    public static final RobotType kDefaultRobot = RobotType.COMPETITION;

    private Constants() {}
}
