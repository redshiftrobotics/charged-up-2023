package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

// The subsystem for controlling the arm
public class Arm extends SubsystemBase {
	// Initializing the motor, encoder, and PID controller
	private final CANSparkMax armMotor;
	private final CANCoder armEncoder;
	private final PIDController armPIDController;
	private double armDegree;
	private double minDegree;
	private double maxDegree;

	// Constructor for the Arm, which takes in ID's for the motor and encoder as well as for the minimum and maximum degrees
	public Arm(int armMotorId, int armEncoderId, double inMinDegree, double inMaxDegree) {
		armPIDController = new PIDController(
				ArmConstants.ARM_PID_P,
				ArmConstants.ARM_PID_I,
				ArmConstants.ARM_PID_D);
		armMotor = new CANSparkMax(armMotorId, MotorType.kBrushless);
		armEncoder = new CANCoder(armEncoderId);
		minDegree = inMinDegree;
		maxDegree = inMaxDegree;
	}

	// Getting the rotation of the encoder
	public double getEncoderRotation() {
		return (armEncoder.getAbsolutePosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO);
	}

	// Setting the desired degree for the arm
	public void setDegree(double desiredDegree) {
		// Limiting the angle the arm can be set to to between the minimum and maximum degrees
		if (desiredDegree > maxDegree) {
			armDegree = maxDegree;
		}
		if (desiredDegree < minDegree) {
			armDegree = minDegree;
		} else {
			armDegree = desiredDegree;
		}
	}

	// Setting the rotation of the arm
	@Override
	public void periodic() {
		armMotor.set(armPIDController.calculate(getEncoderRotation(), armDegree));
	}
}
