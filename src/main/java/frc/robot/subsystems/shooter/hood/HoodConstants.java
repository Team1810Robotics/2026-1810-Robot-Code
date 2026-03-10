package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Translation3d;

public class HoodConstants {
  public static final int HOOD_MOTOR_ID = 16;
  public static final int HOOD_ENCODER_ID = 35;

  public static final double GEAR_RATIO_MOTOR = 8 * 18.2;
  public static final double GEAR_RATIO_ENCODER = 18.2;

  public static final double kP = 300.0;

  public static final double kS = 0.23;

    public static final Translation3d turretToHood = new Translation3d(.114, .078, .074);

}
