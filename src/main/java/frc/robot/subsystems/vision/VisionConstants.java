package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
  public static final String FRONT_LIMELIGHT_NAME = "front_ll";
  public static final String REAR_LIMELIGHT_NAME = "rear_ll";

  public static final Matrix<N3, N1> visionMeasurementStdDevs =
      VecBuilder.fill(
          0.1, 0.1, Double.MAX_VALUE); // 10cm and 10 degrees std dev for vision measurements
}
