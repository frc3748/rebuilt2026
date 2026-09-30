package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    private static final Mode kSimMode = Mode.SIM;
    public static final Mode kMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    private Constants() {}
}
