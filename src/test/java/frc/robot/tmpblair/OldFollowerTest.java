package frc.robot.tmpblair;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

class OldFollowerTest extends BlairHarness {
    @Override
    protected String label() {
        return "old";
    }

    @Override
    protected void setup(RobotState state) {
        Drive drive = state.getDrive();
        var controller = new PPHolonomicDriveController(drive.getConfig().pathTranslationPid, drive.getConfig().pathRotationPid);
        AutoBuilder.configureCustom(path -> new FollowPathCommand(path, drive::getPose, drive::getChassisSpeeds,
                (speeds, ff) -> drive.runVelocity(speeds, ff), controller, drive.getConfig().pathPlannerConfig(),
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, drive),
                drive::getPose, drive::setPose, true);
    }
}
