package frc.robot.robots;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.robots.competition.CompetitionRobot;
import frc.robot.robots.practice.PracticeRobot;

public enum RobotType {
    COMPETITION(CompetitionRobot::new),
    PRACTICE(PracticeRobot::new);

    private final Supplier<RobotDefinition> factory;
    private final String[] roboRioSerials;

    RobotType(Supplier<RobotDefinition> factory, String... roboRioSerials) {
        this.factory = factory;
        this.roboRioSerials = roboRioSerials;
    }

    public RobotDefinition create() {
        return factory.get();
    }

    public static RobotType detect() {
        if (!RobotBase.isReal()) {
            return Constants.kDefaultRobot;
        }
        String serial = RobotController.getSerialNumber();
        for (RobotType type : values()) {
            if (Arrays.asList(type.roboRioSerials).contains(serial)) {
                return type;
            }
        }
        new Alert("Unknown roboRIO serial " + serial + ", running " + Constants.kDefaultRobot + " code", AlertType.kWarning)
                .set(true);
        return Constants.kDefaultRobot;
    }
}
