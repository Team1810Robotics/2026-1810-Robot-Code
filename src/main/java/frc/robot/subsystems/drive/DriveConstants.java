package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;

public class DriveConstants {
  private static final double simTranlationalkP = 0;
  private static final double simTranlationalkI = 0;
  private static final double simTranlationalkD = 0;

  public static final PIDConstants simTranslationPID =
      new PIDConstants(simTranlationalkP, simTranlationalkI, simTranlationalkD);

  private static final double simHeadingkP = 0;
  private static final double simHeadingkI = 0;
  private static final double simHeadingkD = 0;

  public static final PIDConstants simHeadingPID =
      new PIDConstants(simHeadingkP, simHeadingkI, simHeadingkD);
}
