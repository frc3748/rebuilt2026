package frc.robot.robots.competitionv2;

import frc.robot.robots.competition.CompetitionRobot;
import frc.robot.subsystems.drive.DriveConfig;

public class CompetitionV2Robot extends CompetitionRobot {
    @Override
    public String name() {
        return "Competition V2";
    }

    @Override
    public DriveConfig drive() {
        return new CompetitionV2Drive();
    }
}
