package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
	private SparkMax spinMotor;
	private SparkMax kickerMotor;

	public IndexerSubsystem() {
		spinMotor = new SparkMax(IndexerConstants.SPIN_MOTOR, MotorType.kBrushless);
		kickerMotor = new SparkMax(IndexerConstants.KICKER_MOTOR, MotorType.kBrushless);
	}

	public void run(double spinSpeed, double kickSpeed) {
		spinMotor.set(spinSpeed);
		kickerMotor.set(kickSpeed);
	}

	public void fullstop() {
		spinMotor.stopMotor();
		kickerMotor.stopMotor();
	}

	public void spinstop() {
		spinMotor.stopMotor();
	}

	public void feedstop() {
		kickerMotor.stopMotor();
	}

}
