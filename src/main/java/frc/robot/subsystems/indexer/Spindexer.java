package frc.robot.subsystems.indexer;


import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.SpindexerConstants;

public class Spindexer {
    


public class SpindexerSubsystem extends SubsystemBase {
    private SparkMax spinMotor;
    private SparkMax feedMotor;

    // private ColorSensorV3 colorSensor;

    public SpindexerSubsystem() {
        spinMotor = new SparkMax(SpindexerConstants.SPIN_MOTOR, MotorType.kBrushless);

        // colorSensor = new ColorSensorV3(Port.kOnboard);

    //     Shuffleboard.getTab("Intake").addNumber("Distance", () -> getDistance());
    //     Shuffleboard.getTab("Intake").addBoolean("Distance Sensor", () -> colorSensor.isConnected());
    //     Shuffleboard.getTab("Intake").addNumber("Blue", () -> colorSensor.getBlue());
    }

    public void run(double speeds, double speedf) {
        spinMotor.set(speeds);
        feedMotor.set(speedf);
    }

    // public int getDistance() {
    //     return colorSensor.getProximity();
    // }

    public void fullstop() {
       spinMotor.stopMotor(); 
       feedMotor.stopMotor();
    }

    public void spinstop() {
        spinMotor.stopMotor();
    }

    public void feedstop(){
        feedMotor.stopMotor();
    }

}

}
