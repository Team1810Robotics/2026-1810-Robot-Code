package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;

public class DriveConstants {
  private static final double simTranlationalkP = 1.5;
  private static final double simTranlationalkI = 0;
  private static final double simTranlationalkD = 0;

  public static final PIDConstants simTranslationPID =
      new PIDConstants(simTranlationalkP, simTranlationalkI, simTranlationalkD);

  private static final double simHeadingkP = 3.25;
  private static final double simHeadingkI = 0;
  private static final double simHeadingkD = 0;

  public static final PIDConstants simHeadingPID =
      new PIDConstants(simHeadingkP, simHeadingkI, simHeadingkD);

  private static final double realTranlationkP = 1.5;
  private static final double realHeadingkP = 3.25;

  public static final PIDConstants realTranslationConstants = new PIDConstants(realTranlationkP);
  public static final PIDConstants realHeadingConstants = new PIDConstants(realHeadingkP);
}
