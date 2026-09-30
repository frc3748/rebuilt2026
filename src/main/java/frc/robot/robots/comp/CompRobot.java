package frc.robot.robots.comp;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.Superstructure;
import frc.robot.game.FieldConstants;
import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.IntakeComp;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.ShooterComp;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.vision.CameraConfig;

public class CompRobot extends RobotDefinition {
    @Override
    public String name() {
        return "Comp";
    }

    @Override
    public DriveConfig drive() {
        return new CompDrive();
    }

    @Override
    public CameraConfig[] cameras() {
        return new CameraConfig[] { chassisCamera(), shooterCamera(), fuelCamera() };
    }

    protected CameraConfig chassisCamera() {
        return new CameraConfig("Chassis Camera", "limelight", CameraConfig.Type.LIMELIGHT_4)
                .robotToCamera(new Transform3d(
                        new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0.75), Units.inchesToMeters(5.75)),
                        new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180))));
    }

    protected CameraConfig shooterCamera() {
        return new CameraConfig("Shooter Camera", "limelight-turret", CameraConfig.Type.LIMELIGHT_4)
                .robotToCamera(shooter().shooterToRobotCenter.plus(new Transform3d(
                        new Translation3d(Units.inchesToMeters(4.594), Units.inchesToMeters(4.270), Units.inchesToMeters(4.181)),
                        new Rotation3d(0, Units.degreesToRadians(-20.5), 0))))
                .reportedPoseOffset(new Transform2d(Units.inchesToMeters(4.594), Units.inchesToMeters(4.270), Rotation2d.kZero))
                .stdDevFactor(1.3);
    }

    protected CameraConfig fuelCamera() {
        return new CameraConfig("Fuel Camera", "limelight-fuel", CameraConfig.Type.LIMELIGHT_3)
                .robotToCamera(new Transform3d(
                        new Translation3d(Units.inchesToMeters(12), 0, Units.inchesToMeters(20)),
                        new Rotation3d(0, Units.degreesToRadians(20), 0)))
                .detector(0)
                .objectHeight(FieldConstants.FUEL_DIAMETER.in(Meters) / 2);
    }

    protected IntakeConstants intake() {
        return new IntakeConstants();
    }

    protected FlywheelConstants flywheel() {
        return new FlywheelConstants();
    }

    protected HoodConstants hood() {
        return new HoodConstants();
    }

    protected HopperConstants hopper() {
        return new HopperConstants();
    }

    protected KickerConstants kicker() {
        return new KickerConstants();
    }

    @Override
    public Superstructure createSuperstructure(RobotState state) {
        return new Superstructure(state)
                .withShooter(new ShooterComp(state, flywheel(), hood(), hopper(), kicker()))
                .withIntake(new IntakeComp(state, intake()));
    }
}
