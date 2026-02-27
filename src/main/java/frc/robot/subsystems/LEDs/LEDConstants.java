package frc.robot.subsystems.LEDs;

//Its GRB for some reason 

public final class LEDConstants {
  public static final int[] RED = {0, 255, 0}; // GRB values
  public static final int[] YELLOW = {255, 255, 0}; // GRB values
  public static final int[] WHITE = {255, 255, 255}; // GRB values
  public static final int[] ORANGE = {128, 255, 0}; // GRB values
  public static final int[] GREEN = {0, 255, 0}; // GRB values
  public static final int[] BLUE = {0, 0, 255}; // GRB values
  public static final int[] PURPLE = {0, 255, 255}; // GRB values

  public static final int CANDLE_ID = 35; // CANdle ID (Depends on wiring)

  public static final int NUM_LEDS = 100; // TODO: Find Actual LED amount

  public static final int LED_Brightness = 100; // Brightness level (0-255) (MAX is VERY bright)
}
