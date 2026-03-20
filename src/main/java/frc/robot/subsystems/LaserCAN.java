package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain.DriveConstants;

public class LaserCAN extends SubsystemBase {
    RobotContainer robot;
    List<LaserCan> LCs = new ArrayList<>();

    private final ProfiledPIDController controller = new ProfiledPIDController(
                        DriveConstants.kDriveToPointHeadingP,
                        0.0,
                        0.0, 
        new TrapezoidProfile.Constraints(1.0, 0.5)); // 1 m/s, 0.5 m/s²

    public LaserCAN(RobotContainer robot) {
        this.robot = robot;

        LCs.add(new LaserCan(0));
        LCs.add(new LaserCan(1));

        LaserCan LCL = LCs.get(0);
        try {
            LCL.setRangingMode(LaserCan.RangingMode.SHORT);
            LCL.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 8, 8, 16));
            LCL.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        }
        catch (ConfigurationFailedException e) {
            System.out.println("LaserCan Configuration failed! " + e);
        }
    
        LaserCan LCR = LCs.get(1);
        try {
            LCR.setRangingMode(LaserCan.RangingMode.SHORT);
            LCR.setRegionOfInterest(new LaserCan.RegionOfInterest(12, 8, 8, 16));
            LCR.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        }
        catch (ConfigurationFailedException e) {
            System.out.println("LaserCan Configuration failed! " + e);
        }
    }

    public double getDistance(int ID) {
        LaserCan.Measurement measurement = LCs.get(ID).getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        }

        return -1; 
    }

    public void moveRobotToLocation() {
        double leftDist = getDistance(0);
        double rightDist = getDistance(1);
        final ProfiledPIDController climbAlignController = new ProfiledPIDController(
                        DriveConstants.kDriveToPointHeadingP,
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                        DriveConstants.kMaxAngularSpeed,
                                        DriveConstants.kMaxAngularAcceleration),
                        0.02);
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
        double tolerance = 40;
        double targetDistance = 10;

        double forwardSpeed = 10;
        double strafeSpeed = 10; 
        
        double rotationSpeed = Units.degreesToRadians(360); 

        SmartDashboard.putNumber("LaserCAN/0CAN", leftDist);
        SmartDashboard.putNumber("LaserCAN/1CAN", rightDist);

        if (leftDist != -1 || rightDist != -1) {
            if (leftDist > targetDistance || rightDist > targetDistance) {
                double targetPosition = Math.max(leftDist, rightDist) - targetDistance; // meters
                robot.drive.setRobotSpeeds(new ChassisSpeeds(forwardSpeed, 0, 0));
                SmartDashboard.putString("LaserCAN/Command", "Drive Forward");
                climbAlignController.reset(
                                currentPose.getRotation().getRadians(),
                                robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
                
            } else{
                if (leftDist == -1 && rightDist != -1) {
                    robot.drive.setRobotSpeeds(new ChassisSpeeds(0, strafeSpeed, 0));
                    SmartDashboard.putString("LaserCAN/Command", "Drive Left");
                } else if (rightDist == -1 && leftDist != -1) {
                    robot.drive.setRobotSpeeds(new ChassisSpeeds(0, -strafeSpeed, 0));
                    SmartDashboard.putString("LaserCAN/Command", "Drive Right");
                } else if (rightDist != -1 && leftDist != -1) {
                    if (leftDist + tolerance < rightDist) {
                        robot.drive.setRobotSpeeds(new ChassisSpeeds(0, 0, rotationSpeed));
                        SmartDashboard.putString("LaserCAN/Command", "Turn Left");
                    } else if (rightDist + tolerance < leftDist) {
                        robot.drive.setRobotSpeeds(new ChassisSpeeds(0, 0, -rotationSpeed));
                        SmartDashboard.putString("LaserCAN/Command", "Turn Right");
                    } 
                } else {
                    robot.drive.setRobotSpeeds(new ChassisSpeeds(0, 0, 0));
                    SmartDashboard.putString("LaserCAN/Command", "Align complete");
                }
            }
        }

        // if (leftDist == -1 && rightDist != -1) {
        //     robot.drive.setRobotSpeeds(new ChassisSpeeds(strafeSpeed, 0, 0));
        //     SmartDashboard.putString("LaserCAN/Command", "Drive Left");
        // } else if (rightDist == -1 && leftDist != -1) {
        //     robot.drive.setRobotSpeeds(new ChassisSpeeds(-strafeSpeed, 0, 0));
        //     SmartDashboard.putString("LaserCAN/Command", "Drive Right");
        // } else if (rightDist != -1 && leftDist != -1) {
        //     if (Math.abs(leftDist - rightDist) < tolerance) {
        //         robot.drive.setRobotSpeeds(new ChassisSpeeds(0, forwardSpeed, 0));
        //         SmartDashboard.putString("LaserCAN/Command", "Drive Forward");
        //     } else if (leftDist < rightDist) {
        //         robot.drive.setRobotSpeeds(new ChassisSpeeds(0, 0, rotationSpeed));
        //         SmartDashboard.putString("LaserCAN/Command", "Turn Left");
                
        //     } else {
        //         robot.drive.setRobotSpeeds(new ChassisSpeeds(0, 0, -rotationSpeed));
        //         SmartDashboard.putString("LaserCAN/Command", "Turn Right");
        //     } 
        // } else {
        //     robot.drive.setRobotSpeeds(new ChassisSpeeds(0, forwardSpeed, 0));
        //     SmartDashboard.putString("LaserCAN/Command", "Drive Forward");
        // }
    }

    @Override
    public void periodic() {
        moveRobotToLocation();
    }
}