package frc.robot.commands.autos;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;

public class MeasureSlipCurrent extends MeasureAuto {
    private static final double kGravity = 9.80665;
    private static final double kPointSeconds = 0.5;
    private static final double kRampVoltsPerSec = 1.0;
    private static final double kMaxVolts = 10.0;
    private static final double kTestCurrentLimit = 80.0;
    private static final double kSlipSpeed = 0.3;
    private static final double kMinWallAmps = 15.0;
    private static final double kLimitFraction = 0.9;
    private static final double kSameAmps = 3.0;

    private final Timer timer = new Timer();
    private double peakAmps;
    private double slipVolts;
    private boolean spun;

    public MeasureSlipCurrent(RobotState state) {
        super(state, "SlipCurrent", "Measure: slip current");
    }

    @Override
    protected Command measure() {
        Drive drive = state.getDrive();
        return Commands.sequence(
                Commands.runOnce(() -> {
                    peakAmps = 0.0;
                    slipVolts = 0.0;
                    spun = false;
                }),
                Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(kPointSeconds),
                Commands.runOnce(() -> {
                    drive.liftDriveCurrentLimit(kTestCurrentLimit);
                    timer.restart();
                }),
                Commands.run(this::push, drive).until(() -> spun || volts() >= kMaxVolts),
                Commands.runOnce(() -> drive.runCharacterization(0.0), drive),
                Commands.runOnce(this::finish))
                .finallyDo(drive::restoreDriveCurrentLimit);
    }

    private double volts() {
        return timer.get() * kRampVoltsPerSec;
    }

    private void push() {
        Drive drive = state.getDrive();
        if (Math.abs(drive.getWheelSpeedMetersPerSec()) >= kSlipSpeed) {
            spun = true;
            return;
        }
        peakAmps = Math.max(peakAmps, drive.getDriveCurrentAmps());
        slipVolts = volts();
        drive.runCharacterization(slipVolts);
    }

    private void finish() {
        DriveConfig config = state.getDrive().getConfig();
        if (spun && peakAmps < kMinWallAmps) {
            fail("It drove off instead of pushing. Put the front bumper flat against a wall.");
            return;
        }
        Map<String, Double> measured = new LinkedHashMap<>();
        measured.put("PeakAmps", peakAmps);
        measured.put("Volts", slipVolts);
        if (!spun) {
            measured.put("Slipped", 0.0);
            report(Outcome.SAME, String.format(Locale.ROOT,
                    "The wheels held up to %.0f A without spinning, so the %d A limit can't make them slip", peakAmps,
                    config.driveCurrentLimit), measured, Map.of());
            return;
        }
        double wheelForce = peakAmps * config.driveGearbox.KtNMPerAmp * config.driveReduction / config.wheelRadiusMeters;
        double grip = wheelForce / (config.robotMassKg * kGravity / 4.0);
        double limit = Math.floor(peakAmps * kLimitFraction);
        measured.put("Slipped", 1.0);
        measured.put("SlipAmps", peakAmps);
        measured.put("Grip", grip);
        measured.put("RecommendedLimit", limit);
        boolean same = Math.abs(limit - config.driveCurrentLimit) <= kSameAmps;
        String summary = String.format(Locale.ROOT,
                "The wheels spin at %.0f A (grip about %.2f). Limit them to %.0f A; the code says %d A", peakAmps, grip, limit,
                config.driveCurrentLimit);
        report(same ? Outcome.SAME : Outcome.CHANGED, summary, measured, Map.of("Drive/Current Limit", limit));
    }
}
