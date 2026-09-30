package frc.robot.game;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameState {
    private static final double kTransitionStart = 130;
    private static final double kEndGameStart = 30;
    private static final double kShiftLength = 25;

    private String phase = "No Game State";
    private double secondsUntilShift;
    private boolean hubActive = true;
    private boolean wonAuto;

    public void update() {
        String message = DriverStation.getGameSpecificMessage();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        char autoWinner = message.isEmpty() ? ' ' : message.charAt(0);
        double matchTime = DriverStation.getMatchTime();
        boolean inTransition = matchTime >= kTransitionStart;
        boolean inEndGame = matchTime <= kEndGameStart;
        int shift = 4 - (int) ((matchTime - kEndGameStart) / kShiftLength);

        if (DriverStation.isAutonomous()) {
            phase = "Autonomous";
            secondsUntilShift = 0;
        } else if (inTransition) {
            phase = "Transition";
            secondsUntilShift = matchTime - kTransitionStart;
        } else if (inEndGame) {
            phase = "End Game";
            secondsUntilShift = matchTime;
        } else {
            phase = shift >= 1 && shift <= 4 ? "Shift " + shift : "Teleop";
            secondsUntilShift = (matchTime - kEndGameStart) % kShiftLength;
        }

        boolean shiftsKnown = alliance.isPresent() && !message.isEmpty() && !inTransition && !inEndGame
                && DriverStation.isTeleop();
        if (shiftsKnown) {
            char ourColor = alliance.get() == Alliance.Red ? 'R' : 'B';
            boolean winnerKnown = autoWinner == 'B' || autoWinner == 'R';
            hubActive = !winnerKnown || (shift % 2 == 0) == (ourColor == autoWinner);
            wonAuto = autoWinner == ourColor;
        } else {
            hubActive = true;
        }
    }

    public String getPhase() {
        return phase;
    }

    public double getSecondsUntilShift() {
        return secondsUntilShift;
    }

    public boolean isHubActive() {
        return hubActive;
    }

    public boolean wonAuto() {
        return wonAuto;
    }
}
