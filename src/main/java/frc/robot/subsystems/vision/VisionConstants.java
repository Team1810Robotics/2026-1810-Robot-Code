package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
  public static final String LEFT_LIMELIGHT_NAME = "limelight-left";
  public static final String RIGHT_LIMELIGHT_NAME = "limelight-right";

  public static final Matrix<N3, N1> visionMeasurementStdDevs =
      VecBuilder.fill(
          0.1, 0.1, Double.MAX_VALUE); // 10cm and 10 degrees std dev for vision measurements
}
