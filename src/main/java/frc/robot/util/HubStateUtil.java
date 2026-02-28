package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;

/** Utility class to determine the current "hub state" based on match time and auto results. */
public class HubStateUtil {
  public enum Shift {
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    DISABLED
  }

  public record HubState(Shift shift, double remainingTime, boolean isActive) {}

  private static Map<Shift, Double> shiftStartTimes =
      Map.of(
          Shift.TRANSITION, 140.0,
          Shift.SHIFT_1, 130.0,
          Shift.SHIFT_2, 105.0,
          Shift.SHIFT_3, 80.0,
          Shift.SHIFT_4, 55.0,
          Shift.ENDGAME, 30.0,
          Shift.DISABLED, 0.0);

  private static Map<Shift, Double> shiftEndTimes =
      Map.of(
          Shift.TRANSITION, 130.0,
          Shift.SHIFT_1, 105.0,
          Shift.SHIFT_2, 80.0,
          Shift.SHIFT_3, 55.0,
          Shift.SHIFT_4, 30.0,
          Shift.ENDGAME, 0.0,
          Shift.DISABLED, 0.0);

  public static Alliance getAutoWinner() {
    String message = DriverStation.getGameSpecificMessage();

    if (message.length() > 0) {
      char winner = message.charAt(0);
      if (winner == 'R') {
        return Alliance.Red;
      } else if (winner == 'B') {
        return Alliance.Blue;
      }
    }

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    return alliance;
  }

  public static Map<Shift, Boolean> getActiveSchedule(boolean wonAuto) {
    if (wonAuto) {
      return Map.of(
          Shift.AUTO, true,
          Shift.TRANSITION, true,
          Shift.SHIFT_1, false,
          Shift.SHIFT_2, true,
          Shift.SHIFT_3, false,
          Shift.SHIFT_4, true,
          Shift.ENDGAME, true,
          Shift.DISABLED, false);
    } else {
      return Map.of(
          Shift.AUTO, true,
          Shift.TRANSITION, true,
          Shift.SHIFT_1, true,
          Shift.SHIFT_2, false,
          Shift.SHIFT_3, true,
          Shift.SHIFT_4, false,
          Shift.ENDGAME, true,
          Shift.DISABLED, false);
    }
  }

  public static Shift getCurrentShift() {
    if (DriverStation.isDisabled()) {
      return Shift.DISABLED;
    }

    if (DriverStation.isAutonomous()) {
      return Shift.AUTO;
    }

    double time = DriverStation.getMatchTime();

    try {
      for (Shift state : Shift.values()) {
        if (shiftStartTimes.containsKey(state) && shiftEndTimes.containsKey(state)) {
          double startTime = shiftStartTimes.get(state);
          double endTime = shiftEndTimes.get(state);
          if (time <= startTime && time > endTime) {
            return state;
          }
        }
      }
    } catch (NullPointerException npe) {
      return Shift.AUTO;
    }

    return Shift.SHIFT_1;
  }

  public static HubState getCurrentHubState() {
    Alliance autoWinner = getAutoWinner();
    boolean wonAuto = autoWinner == DriverStation.getAlliance().orElse(Alliance.Blue);
    double time = DriverStation.getMatchTime();

    Map<Shift, Boolean> schedule = getActiveSchedule(wonAuto);

    Shift currentShift = getCurrentShift();

    double endTime = shiftEndTimes.getOrDefault(currentShift, 0.0);

    double remainingTime = time - endTime;
    boolean isActive = schedule.get(currentShift);

    return new HubState(currentShift, remainingTime, isActive);
  }

  public static void log() {
    HubState state = getCurrentHubState();

    if (state == null) {
      return;
    }

    DogLog.log("Hub/Shift", state.shift.name());
    DogLog.log("Hub/Shift Remaining Time", state.remainingTime);
    DogLog.log("Hub/Hub Active", state.isActive());
  }
}