package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.ArmConstants;

// The subsystem for controlling the top arm
public class TopArm extends SubsystemBase {
	// Initializing the motor, encoder, and PID controller

	// private final Joystick joy = new Joystick(0);

	private final CANSparkMax armMotor;
	private final CANCoder armEncoder;
	private final PIDController armPIDController;
	private double armDegree;
	private double minDegree;
	private double maxDegree;

	private final ArmFeedforward feedForward = new ArmFeedforward(
			ArmConstants.TOP_ARM_PID_S,
			ArmConstants.TOP_ARM_PID_G,
			ArmConstants.TOP_ARM_PID_V,
			ArmConstants.TOP_ARM_PID_A);

	// Constructor for the top arm, which takes in ID's for the motor and encoder as well as for the minimum and maximum degrees
	public TopArm(int armMotorId, int armEncoderId, double inMinDegree, double inMaxDegree) {
		armPIDController = new PIDController(
				ArmConstants.TOP_ARM_PID_P,
				ArmConstants.TOP_ARM_PID_I,
				ArmConstants.TOP_ARM_PID_D);
		armMotor = new CANSparkMax(armMotorId, MotorType.kBrushless);
		armEncoder = new CANCoder(armEncoderId);
		minDegree = inMinDegree;
		maxDegree = inMaxDegree;
	}

	// Getting the rotation of the encoder
	public double getEncoderRotation() {
		return (armEncoder.getAbsolutePosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO);
	}

	// Setting the desired degree for the top arm
	public void setDegree(double desiredDegree) {
		// Limiting the angle the arm can be set to to between the minimum and maximum degrees
		// Minimum degree will not always be set to zero. 
		if (minDegree + desiredDegree > maxDegree) {
			armDegree = maxDegree;
		}
		if (minDegree + desiredDegree < minDegree) {
			armDegree = minDegree;
		} else {
			armDegree = minDegree + desiredDegree;
		}
	}

	// Setting the rotation of the top arm
	// Uncomment the bottom comment and comment the top code to slow down the arm for testing
	@Override
	public void periodic() {
		armMotor.set(armPIDController.calculate(getEncoderRotation() + feedForward.calculate(
				ArmConstants.TOP_ARM_FEEDFORWARD_POS,
				ArmConstants.TOP_ARM_FEEDFORWARD_VEL,
				ArmConstants.TOP_ARM_FEEDFORWARD_ACCEL), armDegree));

		/*
		armMotor.set(armPIDController.calculate(getEncoderRotation() * 0.05, armDegree));
		*/

		// if (joy.getRawButton(3)) {
		// 	armMotor.set(0.01);
		// }
		// if (joy.getRawButton(4)) {
		// 	armMotor.set(-0.01);
		// }
	}
}
