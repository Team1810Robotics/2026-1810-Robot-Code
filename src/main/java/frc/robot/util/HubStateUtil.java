package frc.robot.util;

import java.util.Map;
import java.util.Optional;

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
        HubStateEnum.TRANSITION, 0.0,
        HubStateEnum.SHIFT_1, 10.0,
        HubStateEnum.SHIFT_2, 35.0,
        HubStateEnum.SHIFT_3, 60.0,
        HubStateEnum.SHIFT_4, 85.0,
        HubStateEnum.ENDGAME, 110.0
     );

     private static Map<HubStateEnum, Double> shiftEndTimes = Map.of(
        HubStateEnum.TRANSITION, 10.0,
        HubStateEnum.SHIFT_1, 35.0,
        HubStateEnum.SHIFT_2, 60.0,
        HubStateEnum.SHIFT_3, 85.0,
        HubStateEnum.SHIFT_4, 110.0,
        HubStateEnum.ENDGAME, 140.0
     );

    private static double autoDuration = 20.0;
    private static double teleopDuration = 140.0;

    private static Timer shiftTimer = new Timer();

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

    public static HubState getCurrentHubState() {
        Alliance autoWinner = getAutoWinner();
        boolean wonAuto = autoWinner == DriverStation.getAlliance().orElse(Alliance.Blue);
        double time = Timer.getMatchTime();

        return new HubState(HubStateEnum.DISABLED, 0, 0, false);
    }

    public static Map<HubStateEnum, Boolean> getSchedule(boolean wonAuto) {
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


}
