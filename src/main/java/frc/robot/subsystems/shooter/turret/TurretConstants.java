package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final int TURRET_MOTOR_ID = 17;
  public static final int TURRET_ENCODER_ID = 1;

  public static final double ROBOT_RELATIVE_OFFSET_DEG = 150;

  public static final double GEAR_RATIO = 32.0;

  public static final Angle MIN_ANGLE = Degrees.of(-150);
  public static final Angle MAX_ANGLE = Degrees.of(150);

  public static final double ENCODER_OFFSET = .491;

  public static final double kP = 4.0;
  public static final double kS = 0.225;
  public static final double kD = 0.002;
  public static final double kV = 2.5;

  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.5; // rot/s
  public static final double MOTION_MAGIC_ACCELERATION = 4; // rot/s²
  public static final double MOTION_MAGIC_JERK = 0.0; // 0 = trapezoidal profile

  public static final Transform3d ROBOT_TO_TURRET =
      new Transform3d(-0.128, -0.128, .28, Rotation3d.kZero);
}
