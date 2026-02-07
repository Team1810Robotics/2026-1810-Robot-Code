package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.util.Units;

public class IntakeSubsystem extends SubsystemBase {
	private SparkMax leftMotor;
	private SparkMax rightMotor;
	private DutyCycleEncoder leftEncoder;
	private DutyCycleEncoder rightEncoder;
	private final PIDController intakePIDController;

	public IntakeSubsystem() {
		leftMotor = new SparkMax(IntakeConstants.LEFT_MOTOR, MotorType.kBrushless);
		rightMotor = new SparkMax(IntakeConstants.RIGHT_MOTOR, MotorType.kBrushless);
		
		leftEncoder = new DutyCycleEncoder(IntakeConstants.LEFT_ENCODER_ID);
		rightEncoder = new DutyCycleEncoder(IntakeConstants.RIGHT_ENCODER_ID);
		
		intakePIDController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
	}

	// public void run(double motorSpeed) {
	// 	leftMotor.set(motorSpeed);
	// 	rightMotor.set(-motorSpeed);
	// }

	    public void run(double setpoint) {
        currentSetpoint = setpoint;
        if (leftEncoder.isConnected() && rightEncoder.isConnected()){
            double output = intakePIDController.calculate(getMeasurement(), setpoint);
            leftMotor.set(-output);
			rightMotor.set(output);
        } else {
            System.out.println("Arm Encoder Disconnected");
            fullstop();
            // Elastic.sendNotification(notification.withAutomaticHeight());
        }
    }

	public void fullstop() {
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	public double getLeftMeasurement() {
        double position = leftEncoder.get() - IntakeConstants.LEFT_ENCODER_OFFSET; 
        double degrees = Units.rotationsToDegrees(position);
       
        return degrees; 
    }

	public double getRightMeasurement() {
        double position = rightEncoder.get() - IntakeConstants.RIGHT_ENCODER_OFFSET; 
        double degrees = Units.rotationsToDegrees(position);
       
        return degrees; 
    }



}
