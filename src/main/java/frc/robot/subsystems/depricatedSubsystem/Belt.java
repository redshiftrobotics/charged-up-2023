package frc.robot.subsystems.depricatedSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Belt extends SubsystemBase {
	private final CANSparkMax BeltMotor;

	public Belt(int motorId) {
		BeltMotor = new CANSparkMax(motorId, MotorType.kBrushless);
	}

	//stops the belt motor
	public void setBeltStop() {
		BeltMotor.set(0);
	}

	//Brings items up the belt using the motor
	public void setBeltIn() {
		BeltMotor.set(1);
	}

	//if an item is stuck, use this to reverce the belt
	public void setBeltOut() {
		BeltMotor.set(-1);
	}
}
