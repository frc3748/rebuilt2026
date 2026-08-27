package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.state.StateMachine;
import frc.robot.subsystems.shooter.dumper.Dumper;
import frc.robot.subsystems.shooter.dumper.DumperIO;
public class Shooter extends StateMachine<Shooter.State> {
    

    private Hood hood;
    private Flywheel flywheel;
    private RobotState state;
    private Dumper dumper;

    public Shooter(RobotState state, DumperIO dumperIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
        super("Shooter", State.UNDETERMINED, State.class);

        dumper = new Dumper(dumperIO, state);
   
        hood = new Hood(hoodIO, state);
        flywheel = new Flywheel(flywheelIO, state);
        this.state = state;

        registerStateTransitions();
        registerStateCommands();

        addChildSubsystem(dumper);
        addChildSubsystem(hood);
        addChildSubsystem(flywheel);
        enable();
    }

    public void registerStateTransitions() {
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.SHOOTING, State.PASSING, State.OUTTAKE);
    }

    public void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.IDLE);
            flywheel.requestTransition(Flywheel.State.IDLE);
            hood.requestTransition(Hood.State.IDLE);
            state.getHopper().requestTransition(Hopper.State.IDLE);
            state.getKicker().requestTransition(Kicker.State.IDLE);
        }));

        registerStateCommand(State.HUB_TRACKING, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.HUB_DUMPING);
            flywheel.requestTransition(Flywheel.State.TRACKING);
            hood.requestTransition(Hood.State.HUB_TRACKING);
            state.getHopper().requestTransition(Hopper.State.IDLE);
            state.getKicker().requestTransition(Kicker.State.IDLE);
        }));

        registerStateCommand(State.PASS_TRACKING, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.PASS_DUMPING);
            flywheel.requestTransition(Flywheel.State.TRACKING);
            hood.requestTransition(Hood.State.PASS_TRACKING);
            state.getHopper().requestTransition(Hopper.State.IDLE);
            state.getKicker().requestTransition(Kicker.State.IDLE);
        }));

        registerStateCommand(State.SHOOTING, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.HUB_DUMPING);
            flywheel.requestTransition(Flywheel.State.SHOOT);
            hood.requestTransition(Hood.State.HUB_TRACKING);
            state.getHopper().requestTransition(Hopper.State.SHOOT);
            state.getKicker().requestTransition(Kicker.State.SHOOT);
        }));

        registerStateCommand(State.PASSING, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.PASS_DUMPING);
            flywheel.requestTransition(Flywheel.State.PASS);
            hood.requestTransition(Hood.State.PASS_TRACKING);
            state.getHopper().requestTransition(Hopper.State.SHOOT);
            state.getKicker().requestTransition(Kicker.State.SHOOT);
        }));

        registerStateCommand(State.OUTTAKE, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.IDLE);
            flywheel.requestTransition(Flywheel.State.IDLE);
            hood.requestTransition(Hood.State.IDLE);
            state.getHopper().requestTransition(Hopper.State.OUTAKE);
            state.getKicker().requestTransition(Kicker.State.OUTAKE);
        }));

        registerStateCommand(State.TUNING, new InstantCommand(() -> {
            dumper.requestTransition(Dumper.State.TUNING);
            flywheel.requestTransition(Flywheel.State.TUNING);
            hood.requestTransition(Hood.State.TUNING);
        }));
    }

    public Dumper getDumper() {
        return dumper;
    }

    public Hood getHood() {
        return hood;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        HUB_TRACKING,
        PASS_TRACKING,
        SHOOTING,
        PASSING,
        OUTTAKE,
        TUNING,

        // flags

    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }
}
