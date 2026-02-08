package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final int TURRET_MOTOR_ID = 1; // Replace with actual motor ID
  public static final int TURRET_ENCODER_ID = 0; // Replace with actual encoder ID

  public static final double GEAR_RATIO = 1.0; // Replace with actual gear ratio

  public static final Angle MIN_ANGLE = Degrees.of(2.5); // Minimum angle in degrees
  public static final Angle MAX_ANGLE = Degrees.of(357.5); // Maximum angle in degrees
}
