package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubStatus {
  public boolean hubActive = true;
  private boolean known = false;

  public void update() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();
    double t = DriverStation.getMatchTime();

    if (allianceOpt.isEmpty()) {
      hubActive = true;
      known = false;
      publish(0);
      return;
    }
    Alliance alliance = allianceOpt.get();

    // AUTO
    if (DriverStation.isAutonomous()) {
      hubActive = true;
      known = true;
      publish(0);
      return;
    }

    // TELEOP
    if (!DriverStation.isTeleop() || t < 0) {
      hubActive = true;
      known = false;
      publish(0);
      return;
    }

    if (t > 130.0 || t <= 30.0) {
      hubActive = true;
      known = true;
      publish(0);
      return;
    }

    int shift;
    if (t > 105.0) shift = 1;
    else if (t > 80.0) shift = 2;
    else if (t > 55.0) shift = 3;
    else shift = 4;

    if (gameData == null || gameData.isEmpty()) {
      hubActive = true;
      known = false;
      publish(shift);
      return;
    }

    char c = gameData.charAt(0);
    if (c != 'R' && c != 'B') {
      hubActive = true;
      known = false;
      publish(shift);
      return;
    }

    Alliance firstInactiveAlliance = (c == 'R') ? Alliance.Red : Alliance.Blue;

    boolean isEvenShift = (shift % 2) == 0;
    Alliance activeAlliance =
        isEvenShift ? firstInactiveAlliance
                    : (firstInactiveAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red);

    hubActive = (alliance == activeAlliance);
    known = true;
    publish(shift);
  }

  public boolean isHubActive() { return hubActive; }
  public boolean isKnown() { return known; }

  private void publish(int shift) {
    SmartDashboard.putBoolean("Hub/Active", hubActive);
    SmartDashboard.putBoolean("Hub/Known", known);
    SmartDashboard.putNumber("Hub/Shift", shift);
  }
}
