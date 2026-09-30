package frc.robot.game;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.subsystems.drive.Drive;

public class DashboardManager {
    private final RobotState state;
    private final GameState game;
    private final LoggedDashboardChooser<AutoRoutine> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private AutoRoutine previewed;

    public DashboardManager(RobotState state, GameState game, String robotName, List<AutoRoutine> autos) {
        this.state = state;
        this.game = game;

        autoChooser.addDefaultOption("None", AutoRoutine.none());
        for (String name : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(name, AutoRoutine.pathPlanner(name));
        }
        for (AutoRoutine auto : autos) {
            autoChooser.addOption(auto.name(), auto);
        }

        Drive drive = state.getDrive();
        PathPlannerLogging.setLogActivePathCallback(poses -> {
            if (!poses.isEmpty()) {
                drive.setFieldPoses();
            }
            drive.setFieldPoses("Auto Path", poses);
            Logger.recordOutput("Odometry/Trajectory", poses.toArray(Pose2d[]::new));
        });

        SmartDashboard.putString("Robot/Type", robotName);
    }

    public AutoRoutine getSelectedAuto() {
        AutoRoutine selected = autoChooser.get();
        return selected != null ? selected : AutoRoutine.none();
    }

    public void update() {
        if (DriverStation.isAutonomous()) {
            previewSelectedAuto();
        }

        SmartDashboard.putBoolean("Game/HubActivated", game.isHubActive());
        SmartDashboard.putBoolean("Game/WonAuto", game.wonAuto());
        SmartDashboard.putString("Game/GameState", game.getPhase());
        SmartDashboard.putString("Game/ShiftCountdown", String.format("%.2f", game.getSecondsUntilShift()));
        SmartDashboard.putBoolean("Robot/AutoChoosed", getSelectedAuto().name().toLowerCase().contains("game"));
        Logger.recordOutput("Game/Phase", game.getPhase());
        Logger.recordOutput("Game/HubActive", game.isHubActive());
        Logger.recordOutput("Game/WonAuto", game.wonAuto());
        Logger.recordOutput("Game/DistanceToHub", TrenchZone.getDistanceToClosestShootingPose(state));
    }

    public void clearPreview() {
        previewed = null;
        state.getDrive().setFieldPoses("Auto Path", new ArrayList<>());
        state.getDrive().setFieldPoses();
    }

    private void previewSelectedAuto() {
        AutoRoutine selected = getSelectedAuto();
        if (selected == previewed) {
            return;
        }
        previewed = selected;

        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : selected.previewPaths()) {
            for (Pose2d pose : path.getPathPoses()) {
                poses.add(AllianceFlip.forAlliance(pose));
            }
        }
        state.getDrive().setFieldPoses(poses.toArray(Pose2d[]::new));
    }
}
