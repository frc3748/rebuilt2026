package frc.robot.game;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameState {
    private static final double kTransitionStart = 130;
    private static final double kEndGameStart = 30;
    private static final double kShiftLength = 25;
    private static final double kTeleopLength = 140;

    private String phase = "No Game State";
    private double secondsUntilShift;
    private boolean hubActive = true;
    private boolean hubActiveNext = true;
    private static final String[] kShiftPhases = {"Teleop", "Shift 1", "Shift 2", "Shift 3", "Shift 4"};
    private String[] timeline = new String[0];
    private int timelineFor = Integer.MIN_VALUE;
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
            phase = shift >= 1 && shift <= 4 ? kShiftPhases[shift] : "Teleop";
            secondsUntilShift = (matchTime - kEndGameStart) % kShiftLength;
        }

        boolean shiftsKnown = alliance.isPresent() && !message.isEmpty() && !inTransition && !inEndGame
                && DriverStation.isTeleop();
        if (shiftsKnown) {
            char ourColor = alliance.get() == Alliance.Red ? 'R' : 'B';
            boolean winnerKnown = autoWinner == 'B' || autoWinner == 'R';
            hubActive = !winnerKnown || activeInShift(shift, ourColor, autoWinner);
            wonAuto = autoWinner == ourColor;
        } else {
            hubActive = true;
        }

        boolean winnerKnown = alliance.isPresent() && (autoWinner == 'B' || autoWinner == 'R');
        char ourColor = alliance.map(color -> color == Alliance.Red ? 'R' : 'B').orElse(' ');
        int key = winnerKnown ? ourColor << 16 | autoWinner : -1;
        if (key != timelineFor) {
            timelineFor = key;
            String[] shifts = new String[4];
            for (int index = 1; index <= 4; index++) {
                shifts[index - 1] = !winnerKnown ? "unknown" : activeInShift(index, ourColor, autoWinner) ? "active" : "inactive";
            }
            timeline = new String[] {
                segment("Transition", kTeleopLength, kTransitionStart, "both"),
                segment("1", kTransitionStart, kTransitionStart - kShiftLength, shifts[0]),
                segment("2", kTransitionStart - kShiftLength, kTransitionStart - 2 * kShiftLength, shifts[1]),
                segment("3", kTransitionStart - 2 * kShiftLength, kTransitionStart - 3 * kShiftLength, shifts[2]),
                segment("4", kTransitionStart - 3 * kShiftLength, kEndGameStart, shifts[3]),
                segment("End", kEndGameStart, 0, "both"),
            };
        }

        if (!winnerKnown || DriverStation.isAutonomous() || inEndGame) {
            hubActiveNext = true;
        } else if (inTransition) {
            hubActiveNext = activeInShift(1, ourColor, autoWinner);
        } else {
            hubActiveNext = shift >= 4 || activeInShift(shift + 1, ourColor, autoWinner);
        }
    }

    private static String segment(String label, double from, double to, String state) {
        return label + "\t" + from + "\t" + to + "\t" + state;
    }

    public String[] getTimeline() {
        return timeline;
    }

    private static boolean activeInShift(int shift, char ourColor, char autoWinner) {
        return (shift % 2 == 0) == (ourColor == autoWinner);
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

    public boolean isHubActiveNext() {
        return hubActiveNext;
    }

    public boolean wonAuto() {
        return wonAuto;
    }
}
