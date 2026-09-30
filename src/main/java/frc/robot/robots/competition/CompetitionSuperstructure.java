package frc.robot.robots.competition;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Controls;
import frc.robot.RobotState;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.game.FuelSimulation;
import frc.robot.robots.Superstructure;
import frc.robot.robots.competition.autos.CenterOnlyStarting8;
import frc.robot.robots.competition.autos.CustomAuto;
import frc.robot.robots.competition.autos.DepotOnlyStarting8;
import frc.robot.robots.competition.autos.DepotSideBlair;
import frc.robot.robots.competition.autos.DepotSideBump;
import frc.robot.robots.competition.autos.DepotSideCircuitShoot;
import frc.robot.robots.competition.autos.DepotSideDepotMidHalfSweep;
import frc.robot.robots.competition.autos.DepotSideQuickShoot;
import frc.robot.robots.competition.autos.DepotSideToDepot;
import frc.robot.robots.competition.autos.DepotSideToDepotEndAtMid;
import frc.robot.robots.competition.autos.HpOnlyStarting8;
import frc.robot.robots.competition.autos.HpSideBlair;
import frc.robot.robots.competition.autos.HpSideQuickShoot;
import frc.robot.robots.competition.autos.HpSideToHp;
import frc.robot.robots.competition.autos.HpSideToHpEndAtMid;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.state.StateMachine;

public class CompetitionSuperstructure implements Superstructure {
    private static final int kStartingSimFuel = 8;

    private final RobotState state;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Hopper hopper;
    private final Kicker kicker;
    private final Intake intake;
    private final FuelSimulation fuel;
    private final Shooter shooter;

    public CompetitionSuperstructure(RobotState state) {
        this.state = state;
        flywheel = new Flywheel(state);
        hood = new Hood(state);
        hopper = new Hopper(flywheel::isReady);
        kicker = new Kicker(flywheel::isReady);
        intake = new Intake(state);
        fuel = Constants.kMode == Mode.SIM ? createFuelSimulation() : null;
        shooter = new Shooter(state, flywheel, hood, hopper, kicker, fuel);
    }

    private FuelSimulation createFuelSimulation() {
        DriveConfig drive = state.getDrive().getConfig();
        double halfWidth = (drive.trackWidth + Units.inchesToMeters(6)) / 2.0;
        return new FuelSimulation(
                new FuelSimulation.Robot(drive.trackWidth, drive.wheelBase, drive.bumperHeight, ShooterConstants.kShooterToRobotCenter.getZ()),
                new FuelSimulation.Intake(halfWidth, halfWidth + Units.inchesToMeters(8.5), -halfWidth, halfWidth),
                kStartingSimFuel,
                () -> state.getLatestFieldToRobot().getValue().transformBy(new Transform2d(
                        ShooterConstants.kShooterToRobotCenter.getTranslation().toTranslation2d(), Rotation2d.kZero)),
                state::getLatestDesiredFieldRelativeChassisSpeed,
                () -> intake.getState() == Intake.State.INTAKE);
    }

    @Override
    public List<StateMachine<?>> subsystems() {
        return List.of(shooter, hopper, kicker, intake);
    }

    @Override
    public List<AutoRoutine> autos() {
        return List.of(
                new CenterOnlyStarting8(this),
                new DepotOnlyStarting8(this),
                new DepotSideToDepot(this),
                new DepotSideToDepotEndAtMid(this),
                new HpOnlyStarting8(this),
                new HpSideToHp(this),
                new HpSideToHpEndAtMid(this),
                new DepotSideDepotMidHalfSweep(this),
                new DepotSideQuickShoot(this),
                new HpSideQuickShoot(this),
                new DepotSideCircuitShoot(this),
                new DepotSideBlair(this),
                new HpSideBlair(this),
                new DepotSideBump(this),
                new CustomAuto(this));
    }

    @Override
    public void simulationPeriodic() {
        if (fuel != null) {
            fuel.update();
        }
    }

    @Override
    public void bindControls(Controls controls) {
        bindDriver(controls.driver());
        bindOperator(controls.operator());
    }

    private void bindDriver(CommandXboxController driver) {
        Drive drive = state.getDrive();

        driver.leftTrigger(0.5)
                .onTrue(intake.transitionCommand(Intake.State.INTAKE))
                .onFalse(intake.transitionCommand(Intake.State.IDLE));
        driver.leftBumper().onTrue(intake.transitionCommand(Intake.State.STOW));

        driver.rightTrigger(0.5)
                .onTrue(Commands.sequence(
                        Commands.runOnce(drive::stopWithX),
                        ActionCommands.shootOrPassBasedOnPos(this)))
                .onFalse(ActionCommands.trackBasedOnPos(this));

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
        driver.povUp().whileTrue(ActionCommands.turnToHub(this));
    }

    private void bindOperator(CommandXboxController operator) {
        operator.leftStick().onTrue(Commands.runOnce(this::clearOverrides));

        bindChord(operator, operator.rightStick(),
                () -> {
                    hopper.clearOverride();
                    kicker.clearOverride();
                },
                intake::clearOverride,
                shooter::releaseShot);

        bindChord(operator, operator.leftTrigger(0.5),
                () -> {
                    hopper.setOverride(Hopper.State.IDLE);
                    kicker.setOverride(Kicker.State.IDLE);
                },
                () -> intake.setOverride(intake::rollIn),
                () -> shooter.holdShot(state.getCurrentHubSetpoint(), true));

        bindChord(operator, operator.rightTrigger(0.5),
                () -> {
                    hopper.setOverride(Hopper.State.IDLE);
                    kicker.setOverride(Kicker.State.IDLE);
                },
                () -> intake.setOverride(intake::rollOut),
                () -> shooter.holdShot(state.getCurrentPassSetpoint(), true));

        bindChord(operator, operator.leftBumper(),
                () -> {
                    hopper.setOverride(hopper::feed);
                    kicker.setOverride(kicker::feed);
                },
                () -> intake.setOverride(Intake.State.STOW),
                () -> shooter.holdShot(state.getCurrentHubSetpoint(), false));

        bindChord(operator, operator.rightBumper(),
                () -> {
                    hopper.setOverride(Hopper.State.OUTAKE);
                    kicker.setOverride(Kicker.State.OUTAKE);
                },
                () -> intake.setOverride(Intake.State.INTAKE),
                () -> shooter.holdShot(state.getCurrentPassSetpoint(), false));
    }

    private static void bindChord(CommandXboxController operator, Trigger button, Runnable feedAction,
            Runnable intakeAction, Runnable shooterAction) {
        button.onTrue(Commands.runOnce(() -> {
            if (operator.x().getAsBoolean()) {
                feedAction.run();
            } else if (operator.b().getAsBoolean()) {
                intakeAction.run();
            } else if (operator.a().getAsBoolean()) {
                shooterAction.run();
            }
        }));
    }

    private void clearOverrides() {
        hopper.clearOverride();
        kicker.clearOverride();
        intake.clearOverride();
        shooter.releaseShot();
    }

    public RobotState state() {
        return state;
    }

    public Drive getDrive() {
        return state.getDrive();
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Hood getHood() {
        return hood;
    }

    public Hopper getHopper() {
        return hopper;
    }

    public Kicker getKicker() {
        return kicker;
    }

    public Intake getIntake() {
        return intake;
    }
}
