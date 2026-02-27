package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubStateUtil {
    public enum HubStateEnum {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        DISABLED
    }

    public record HubState(HubStateEnum state, 
                            double elapsedTime,
                            double remainingTime,
                            boolean isActive) {}

     private static Map<HubStateEnum, Double> shiftStartTimes = Map.of(
        HubStateEnum.TRANSITION, 140.0,
        HubStateEnum.SHIFT_1, 130.0,
        HubStateEnum.SHIFT_2, 105.0,
        HubStateEnum.SHIFT_3, 80.0,
        HubStateEnum.SHIFT_4, 55.0,
        HubStateEnum.ENDGAME, 30.0
     );

     private static Map<HubStateEnum, Double> shiftEndTimes = Map.of(
        HubStateEnum.TRANSITION, 130.0,
        HubStateEnum.SHIFT_1, 105.0,
        HubStateEnum.SHIFT_2, 80.0,
        HubStateEnum.SHIFT_3, 55.0,
        HubStateEnum.SHIFT_4, 30.0,
        HubStateEnum.ENDGAME, 0.0
     );

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

    public static Map<HubStateEnum, Boolean> getActiveSchedule(boolean wonAuto) {
        if (wonAuto) {
            return Map.of(
                HubStateEnum.AUTO, true,
                HubStateEnum.TRANSITION, true,
                HubStateEnum.SHIFT_1, false,
                HubStateEnum.SHIFT_2, true,
                HubStateEnum.SHIFT_3, true,
                HubStateEnum.SHIFT_4, false,
                HubStateEnum.ENDGAME, true,
                HubStateEnum.DISABLED, false
            );
        } else {
            return Map.of(
                HubStateEnum.AUTO, true,
                HubStateEnum.TRANSITION, true,
                HubStateEnum.SHIFT_1, true,
                HubStateEnum.SHIFT_2, false,
                HubStateEnum.SHIFT_3, true,
                HubStateEnum.SHIFT_4, false,
                HubStateEnum.ENDGAME, true,
                HubStateEnum.DISABLED, false
            );
        }
    }

    public static HubStateEnum getCurrentHubStateEnum() {
        if (DriverStation.isAutonomous()) {
            return HubStateEnum.AUTO;
        }

        if (DriverStation.isDisabled()) {
            return HubStateEnum.DISABLED;
        }

        double time = DriverStation.getMatchTime();

        for (HubStateEnum state : HubStateEnum.values()) {
            if (shiftStartTimes.containsKey(state) && shiftEndTimes.containsKey(state)) {
                double startTime = shiftStartTimes.get(state);
                double endTime = shiftEndTimes.get(state);
                if (time <= startTime && time > endTime) {
                    return state;
                }
            }
        }

        throw new IllegalStateException("Current time does not fall within any defined HubStateEnum");
    }

    public static HubState getCurrentHubState() {
        Alliance autoWinner = getAutoWinner();
        boolean wonAuto = autoWinner == DriverStation.getAlliance().orElse(Alliance.Blue);
        double time = Timer.getMatchTime();

        Map<HubStateEnum, Boolean> schedule = getActiveSchedule(wonAuto);

        HubStateEnum currentState = getCurrentHubStateEnum();

        double startTime = shiftStartTimes.get(currentState);
        double elapsedTime = startTime - time;
        double remainingTime = time - shiftEndTimes.get(currentState);
        boolean isActive = schedule.get(currentState);

        return new HubState(currentState, elapsedTime, remainingTime, isActive);
    }

    public void log() {
        //TODO: Implement this on a laptop with DogLog
    }

}
