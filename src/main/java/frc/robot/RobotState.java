package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.FieldConstants;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.BallTargetFactory;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.DynamicPathGenerator;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.FuelSim;
import frc.robot.util.MathHelpers;
import frc.robot.util.PassTargetFactory;
import frc.robot.util.RobotTime;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.SimulatedRobotState;
import frc.robot.util.TrenchZone;
import frc.robot.util.state.StateMachine;

public class RobotState extends StateMachine<RobotState.State> {
    public static final double LOOKBACK_TIME = 1.0;
    public static final AtomicBoolean hubActivated = new AtomicBoolean();

    private static final String[] kAutoNames = {
            "Center Only Starting 8 (GAME)",
            "Center Only Starting 8 Climb (GAME)",
            "Depot Only Starting 8 (GAME)",
            "Depot Only Starting 8 Climb (GAME)",
            "Depot Side To Depot (GAME)",
            "Depot Side To Depot Climb (GAME)",
            "Depot Side To Depot End at Mid (GAME)",
            "HP Only Starting 8 (GAME)",
            "HP Only Starting 8 Climb (GAME)",
            "HP Side To HP (GAME)",
            "HP Side To HP Climb (GAME)",
            "HP Side To HP End at Mid (GAME)",
            "Depot Side Depot Mid Half Sweep (GAME)",
            "Depot Side Quick Shoot Climb (GAME)",
            "HP Side Quick Shoot Climb (GAME)",
            "Depot Side Circut Shoot (GAME)",
            "Depot Side Blair (GAME)",
            "HP Side Blair (GAME)",
            "Depot Side Bump (GAME)",
    };

    private final SimulatedRobotState simulatedRobotState = Robot.isSimulation() ? new SimulatedRobotState(this) : null;
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Climb climb;
    private final Hopper hopper;
    private final Intake intake;
    private final Kicker kicker;

    private final Supplier<ShooterSetpoint> hubSupplier;
    private final Supplier<ShooterSetpoint> passSupplier;
    private final CustomAutoBuilder customAutoBuilder;
    private final LoggedDashboardChooser<Command> autoChooser;
    private Command previewedAuto;

    private final FuelSim fuelSim = new FuelSim();
    private double simFuelCount = 8;
    private boolean climbZeroed;

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

    public RobotState() {
        super("RobotState", State.UNDETERMINED, State.class);
        clearBuffers();

        hubSupplier = ShooterSetpoint.hubSetpointSupplier(this);
        passSupplier = ShooterSetpoint.passSetpointSupplier(this);

        drive = switch (Constants.kMode) {
            case REAL -> new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    this);
            case SIM -> new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    this);
            case REPLAY -> new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    this);
        };
        vision = new Vision(this, VisionConstants.kCameras);
        shooter = new Shooter(this);
        climb = new Climb();
        hopper = new Hopper(this);
        intake = new Intake(this);
        kicker = new Kicker(this);

        if (Constants.kMode == Mode.SIM) {
            setupFuelSim();
        }

        customAutoBuilder = new CustomAutoBuilder(this);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        setupAutoChooser();

        CameraServer.startAutomaticCapture();
        setupDriverBindings();
        setupOperatorBindings();
        setupRumble();

        addOmniTransitions(State.SOFT_STOP, State.TRAVERSING, State.AUTO);
        registerStateCommand(State.SOFT_STOP, drive.transitionCommand(Drive.State.IDLE));
        registerStateCommand(State.TRAVERSING, drive.transitionCommand(Drive.State.TRAVERSING));
        registerStateCommand(State.AUTO, drive.transitionCommand(Drive.State.TRAVERSING));

        addChildSubsystem(vision);
        addChildSubsystem(drive);
        addChildSubsystem(shooter);
        addChildSubsystem(climb);
        addChildSubsystem(hopper);
        addChildSubsystem(intake);
        addChildSubsystem(kicker);
        enable();

        Logger.recordOutput("Bumper/Pose", new Pose3d());

        try {
            DynamicPathGenerator.warmupInit();
        } catch (Exception e) {
            Elastic.sendNotification(new Notification()
                    .withTitle("Warmup Command")
                    .withLevel(NotificationLevel.ERROR)
                    .withDescription("Failed to warmup commands"));
        }
    }

    private void setupFuelSim() {
        fuelSim.spawnStartingFuel();
        fuelSim.start();
        fuelSim.enableAirResistance();

        SmartDashboard.putData(Commands.runOnce(() -> {
            fuelSim.clearFuel();
            fuelSim.spawnStartingFuel();
        }).withName("Reset Fuel").ignoringDisable(true));

        fuelSim.registerRobot(
                Meter.of(DriveConstants.trackWidth),
                Meter.of(DriveConstants.wheelBase),
                Meter.of(DriveConstants.kBumperHeight),
                () -> getLatestFieldToRobot().getValue().transformBy(new Transform2d(
                        VisionConstants.kShooterToRobotCenter.getTranslation().toTranslation2d(),
                        Rotation2d.kZero)),
                this::getLatestDesiredFieldRelativeChassisSpeed);

        double halfWidth = Meters.of(DriveConstants.trackWidth + Units.inchesToMeters(6)).div(2).in(Meters);
        fuelSim.registerIntake(
                halfWidth,
                halfWidth + Inches.of(8.5).in(Meters),
                -halfWidth,
                halfWidth,
                () -> intake.getState() == Intake.State.INTAKE,
                () -> simFuelCount++);
    }

    private void setupAutoChooser() {
        for (String name : kAutoNames) {
            AutoCommands.getAutoByName(this, name)
                    .ifPresent(auto -> autoChooser.addOption(name, auto.getCommand(this)));
        }
        autoChooser.addOption("Custom Auto Builder", customAutoBuilder.getCommand(this));

        PathPlannerLogging.setLogActivePathCallback(poses -> {
            if (!poses.isEmpty()) {
                drive.setFieldPoses();
            }
            drive.setFieldPoses("Auto Path", poses);
            Logger.recordOutput("Pathplanner Trajectory", toTransforms(poses));
        });
    }

    private void setupRumble() {
        new Trigger(hubActivated::get).onChange(Commands.startEnd(
                () -> setRumble(1.0),
                () -> setRumble(0.0)).withTimeout(0.5));
    }

    private void setRumble(double value) {
        driver.getHID().setRumble(RumbleType.kBothRumble, value);
        operator.getHID().setRumble(RumbleType.kBothRumble, value);
    }

    private void setupDriverBindings() {
        if (DriverStation.getMatchType() == MatchType.None) {
            driver.povDown().onTrue(Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive)
                    .ignoringDisable(true));
        }

        driver.leftTrigger(0.5)
                .onTrue(intake.transitionCommand(Intake.State.INTAKE))
                .onFalse(intake.transitionCommand(Intake.State.IDLE));
        driver.leftBumper().onTrue(intake.transitionCommand(Intake.State.STOW));

        driver.rightTrigger(0.5)
                .onTrue(Commands.sequence(
                        Commands.runOnce(drive::stopWithX),
                        ActionCommands.shootOrPassBasedOnPos(this)))
                .onFalse(ActionCommands.trackBasedOnPos(this));
        driver.rightBumper()
                .onTrue(drive.transitionCommand(Drive.State.SLOW))
                .onFalse(drive.transitionCommand(Drive.State.TRAVERSING));

        driver.y()
                .whileTrue(ActionCommands.autoClimb(this))
                .onFalse(climb.transitionCommand(Climb.State.STOW));

        driver.a()
                .onTrue(Commands.runOnce(() -> {
                    shooter.requestTransition(Shooter.State.OUTTAKE);
                    intake.requestTransition(Intake.State.OUTAKE);
                }))
                .onFalse(Commands.parallel(
                        Commands.runOnce(() -> intake.requestTransition(Intake.State.IDLE)),
                        ActionCommands.trackBasedOnPos(this)));

        driver.x()
                .whileTrue(ActionCommands.goToFixedPosAndShoot(this))
                .onFalse(Commands.runOnce(shooter::releaseShot));
        driver.b().whileTrue(ActionCommands.shakeIntake(this));

        driver.povLeft().onTrue(drive.transitionCommand(Drive.State.TRAVERSING_AT_ANGLE));
        driver.povRight().onTrue(drive.transitionCommand(Drive.State.TRAVERSING));
        driver.povUp().whileTrue(ActionCommands.turnToHub(this));
    }

    private void setupOperatorBindings() {
        operator.leftStick().onTrue(Commands.runOnce(this::clearOverrides));

        bindOperator(operator.rightStick(),
                climb::clearOverride,
                () -> {
                    hopper.clearOverride();
                    kicker.clearOverride();
                },
                intake::clearOverride,
                shooter::releaseShot);

        bindOperator(operator.leftTrigger(0.5),
                () -> {
                    climb.clearOverride();
                    climb.requestTransition(Climb.State.ZEROING);
                },
                () -> {
                    hopper.setOverride(Hopper.State.IDLE);
                    kicker.setOverride(Kicker.State.IDLE);
                },
                () -> intake.setOverride(intake::rollIn),
                () -> shooter.holdShot(getCurrentHubSetpoint(), true));

        bindOperator(operator.rightTrigger(0.5),
                () -> climb.setOverride(Climb.State.DOWN),
                () -> {
                    hopper.setOverride(Hopper.State.IDLE);
                    kicker.setOverride(Kicker.State.IDLE);
                },
                () -> intake.setOverride(intake::rollOut),
                () -> shooter.holdShot(getCurrentPassSetpoint(), true));

        bindOperator(operator.leftBumper(),
                () -> climb.setOverride(Climb.State.UP),
                () -> {
                    hopper.setOverride(hopper::feed);
                    kicker.setOverride(kicker::feed);
                },
                () -> intake.setOverride(Intake.State.STOW),
                () -> shooter.holdShot(getCurrentHubSetpoint(), false));

        bindOperator(operator.rightBumper(),
                () -> climb.setOverride(Climb.State.STOW),
                () -> {
                    hopper.setOverride(Hopper.State.OUTAKE);
                    kicker.setOverride(Kicker.State.OUTAKE);
                },
                () -> intake.setOverride(Intake.State.INTAKE),
                () -> shooter.holdShot(getCurrentPassSetpoint(), false));
    }

    private void bindOperator(Trigger button, Runnable climbAction, Runnable feedAction, Runnable intakeAction,
            Runnable shooterAction) {
        button.onTrue(Commands.runOnce(() -> {
            if (operator.y().getAsBoolean()) {
                climbAction.run();
            } else if (operator.x().getAsBoolean()) {
                feedAction.run();
            } else if (operator.b().getAsBoolean()) {
                intakeAction.run();
            } else if (operator.a().getAsBoolean()) {
                shooterAction.run();
            }
        }));
    }

    private void clearOverrides() {
        climb.clearOverride();
        hopper.clearOverride();
        kicker.clearOverride();
        intake.clearOverride();
        shooter.releaseShot();
    }

    @Override
    protected void update() {
        if (Math.abs(driver.getRightX()) > 0.1) {
            drive.requestTransition(Drive.State.TRAVERSING);
        }

        Logger.recordOutput("Distance to Hub", TrenchZone.getDistanceToClosestShootingPose(this));

        if (DriverStation.isAutonomous()) {
            previewSelectedAuto();
        }
        updateGameState();

        if (autoChooser.get() != null) {
            SmartDashboard.putBoolean("Robot/AutoChoosed", autoChooser.get().getName().toLowerCase().contains("game"));
        }
    }

    private void previewSelectedAuto() {
        Command selected = autoChooser.get();
        if (selected == null || selected == previewedAuto) {
            return;
        }
        previewedAuto = selected;

        try {
            Optional<AutoCommands.AutoClass> auto = AutoCommands.getAutoByName(this, selected.getName());
            if (auto.isEmpty()) {
                drive.setFieldPoses();
                return;
            }

            List<Pose2d> poses = new ArrayList<>();
            for (PathPlannerPath path : auto.get().getAutoDisplayList(this)) {
                for (Pose2d pose : path.getPathPoses()) {
                    poses.add(isRedAlliance() ? flipPoseForRed(pose) : pose);
                }
            }
            Logger.recordOutput("Auto Trajectory 3D", toTransforms(poses));
            drive.setFieldPoses(poses.toArray(new Pose2d[0]));
        } catch (Exception e) {
            drive.setFieldPoses();
            Elastic.sendNotification(new Notification()
                    .withTitle("Auto Mapping")
                    .withDescription("Unable to add Auto Trajectory")
                    .withLevel(NotificationLevel.ERROR));
        }
    }

    private static Transform3d[] toTransforms(List<Pose2d> poses) {
        return poses.stream()
                .map(pose -> new Transform3d(
                        new Translation3d(pose.getX(), pose.getY(), 0.0),
                        new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())))
                .toArray(Transform3d[]::new);
    }

    private void updateGameState() {
        String message = DriverStation.getGameSpecificMessage();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        char autoWinner = message.length() > 0 ? message.charAt(0) : ' ';
        double matchTime = DriverStation.getMatchTime();
        boolean inTransitionShift = matchTime >= 130;
        boolean inEndGame = matchTime <= 30;
        int currentStage = 4 - (int) ((matchTime - 30) / 25);

        String gameState;
        double secondsUntilShift;
        if (DriverStation.isAutonomous()) {
            gameState = "Autonomous";
            secondsUntilShift = 0;
        } else if (inTransitionShift) {
            gameState = "Transition";
            secondsUntilShift = matchTime - 130;
        } else if (inEndGame) {
            gameState = "End Game";
            secondsUntilShift = matchTime;
        } else {
            gameState = currentStage >= 1 && currentStage <= 4 ? "Shift " + currentStage : "Teleop";
            secondsUntilShift = (matchTime - 30) % 25;
        }

        boolean shiftsKnown = alliance.isPresent() && message.length() > 0 && !inTransitionShift && !inEndGame
                && DriverStation.isTeleop();
        if (shiftsKnown) {
            char myColor = alliance.get() == Alliance.Red ? 'R' : 'B';
            boolean winnerKnown = autoWinner == 'B' || autoWinner == 'R';
            boolean stageEven = currentStage % 2 == 0;
            hubActivated.set(!winnerKnown || stageEven == (myColor == autoWinner));
            SmartDashboard.putBoolean("Game/WonAuto", autoWinner == myColor);
        } else {
            hubActivated.set(true);
        }

        SmartDashboard.putBoolean("Game/HubActivated", hubActivated.get());
        SmartDashboard.putString("Game/GameState", gameState);
        SmartDashboard.putString("Game/ShiftCountdown", String.format("%.2f", secondsUntilShift));
    }

    @Override
    protected void onTeleopStart() {
        setState(State.TRAVERSING);
        drive.setFieldPoses("Auto Path", new ArrayList<>());
        drive.setFieldPoses();
        zeroClimbOnce();
    }

    @Override
    protected void onAutonomousStart() {
        Command selected = autoChooser.get();
        if (selected != null) {
            AutoCommands.getAutoByName(this, selected.getName()).ifPresentOrElse(
                    auto -> registerStateCommand(State.AUTO, auto.getCommand(this)),
                    () -> registerStateCommand(State.AUTO, selected));
        }
        zeroClimbOnce();
        Logger.recordOutput("Auto Trajectory 3D", new Transform3d[] {});
        setState(State.AUTO);
    }

    private void zeroClimbOnce() {
        if (!climbZeroed) {
            climbZeroed = true;
            climb.requestTransition(Climb.State.ZEROING);
        }
    }

    @Override
    protected void determineSelf() {
        setState(State.TRAVERSING);
    }

    public void updateSimulation() {
        if (Constants.kMode == Mode.SIM) {
            fuelSim.updateSim();
        }
    }

    public void updateLogger() {
        logLatest("RobotState/YawAngularVelocity", driveYawAngularVelocity);
        logLatest("RobotState/RollAngularVelocity", driveRollAngularVelocity);
        logLatest("RobotState/PitchAngularVelocity", drivePitchAngularVelocity);
        logLatest("RobotState/AccelX", accelX);
        logLatest("RobotState/AccelY", accelY);
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());
    }

    private static void logLatest(String key, ConcurrentTimeInterpolatableBuffer<Double> buffer) {
        var latest = buffer.getInternalBuffer().lastEntry();
        if (latest != null) {
            Logger.recordOutput(key, latest.getValue());
        }
    }

    public void clearBuffers() {
        fieldToRobot.clear();
        driveYawAngularVelocity.clear();
        fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    public void resetBuffersToPose(Pose2d pose) {
        fieldToRobot.clear();
        fieldToRobot.addSample(Timer.getFPGATimestamp(), pose);
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void addDriveMotionMeasurements(double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        drivePitchRads.addSample(timestamp, pitchRads);
        driveRollRads.addSample(timestamp, rollRads);
        this.accelX.addSample(timestamp, accelX);
        this.accelY.addSample(timestamp, accelY);
        desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public void addVisionMeasurement(VisionMeasurement measurement) {
        if (Constants.kMode == Mode.REAL) {
            drive.addVisionMeasurement(measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
        }
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        fieldToRobot.addSample(RobotTime.getTimestampSeconds(), drive.getPose());
        return fieldToRobot.getLatest();
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if (Constants.kMode != Mode.REAL) {
            return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
        }
        var values = driveYawAngularVelocity.getInternalBuffer().subMap(minTime, maxTime).values();
        return values.stream().max((a, b) -> Double.compare(Math.abs(a), Math.abs(b)));
    }

    public ShooterSetpoint getCurrentHubSetpoint() {
        return hubSupplier.get();
    }

    public ShooterSetpoint getCurrentPassSetpoint() {
        return passSupplier.get();
    }

    public boolean shouldShootHub() {
        double x = getLatestFieldToRobot().getValue().getX();
        return isRedAlliance() ? x >= FieldConstants.HUB_RED.getX() : x <= FieldConstants.HUB_BLUE.getX();
    }

    public Pose2d getDriveAnglePos() {
        Translation3d target = shouldShootHub() ? BallTargetFactory.generate(this) : PassTargetFactory.generate(this);
        return new Pose2d(target.toTranslation2d(), new Rotation2d());
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public static Pose2d flipPoseForRed(Pose2d bluePose) {
        return new Pose2d(
                new Translation2d(
                        VisionConstants.fieldLength - bluePose.getX(),
                        VisionConstants.fieldWidth - bluePose.getY()),
                bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    public Drive getDrive() {
        return drive;
    }

    public Vision getVision() {
        return vision;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Climb getClimb() {
        return climb;
    }

    public Hopper getHopper() {
        return hopper;
    }

    public Intake getIntake() {
        return intake;
    }

    public Kicker getKicker() {
        return kicker;
    }

    public CommandXboxController getController() {
        return driver;
    }

    public CustomAutoBuilder getCustomAutoBuilder() {
        return customAutoBuilder;
    }

    public SimulatedRobotState getSimRobot() {
        return simulatedRobotState;
    }

    public FuelSim getFuelSim() {
        return fuelSim;
    }

    public double getSimFuelCount() {
        return simFuelCount;
    }

    public void setSimFuelCount(double count) {
        simFuelCount = count;
    }

    public enum State {
        UNDETERMINED,
        SOFT_STOP,
        TRAVERSING,
        AUTO
    }
}
