package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

// The subsystem for controlling the bottom arm
public class BottomArm extends SubsystemBase {

	// private final Joystick joy = new Joystick(0);

	// Initializing the motor, encoder, and PID controller
	private final CANSparkMax armMotorOne;
	private final CANSparkMax armMotorTwo;
	private final MotorControllerGroup armMotorControllerGroup;
	private final CANCoder armEncoder;
	private final PIDController armPIDController;
	private double armDegree;
	private double minDegree;
	private double maxDegree;

	private final ArmFeedforward feedForward = new ArmFeedforward(
			ArmConstants.BOTTOM_ARM_PID_S,
			ArmConstants.BOTTOM_ARM_PID_G,
			ArmConstants.BOTTOM_ARM_PID_V,
			ArmConstants.BOTTOM_ARM_PID_A);

	// Constructor for the bottom arm, which takes in ID's for the motor and encoder as well as for the minimum and maximum degrees
	public BottomArm(int armMotorOneId, int armMotorTwoId, int armEncoderId, double inMinDegree, double inMaxDegree) {
		armPIDController = new PIDController(
				ArmConstants.BOTTOM_ARM_PID_P,
				ArmConstants.BOTTOM_ARM_PID_I,
				ArmConstants.BOTTOM_ARM_PID_D);
		armMotorOne = new CANSparkMax(armMotorOneId, MotorType.kBrushless);
		armMotorTwo = new CANSparkMax(armMotorTwoId, MotorType.kBrushless);
		armMotorControllerGroup = new MotorControllerGroup(armMotorOne, armMotorTwo);
		armEncoder = new CANCoder(armEncoderId);
		minDegree = inMinDegree;
		maxDegree = inMaxDegree;
	}

	// Getting the rotation of the encoder
	public double getEncoderRotation() {
		return (armEncoder.getAbsolutePosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO);
	}

	// Setting the desired degree for the bottom arm
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

	// Setting the rotation of the bottom arm
	// Uncomment the bottom comment and comment the top code to slow down the arm for testing
	@Override
	public void periodic() {
		armMotorControllerGroup.set(armPIDController.calculate(getEncoderRotation() + feedForward.calculate(
				ArmConstants.BOTTOM_ARM_FEEDFORWARD_POS,
				ArmConstants.BOTTOM_ARM_FEEDFORWARD_VEL,
				ArmConstants.BOTTOM_ARM_FEEDFORWARD_ACCEL), armDegree));

		/*
		armMotorControllerGroup.set(armPIDController.calculate(getEncoderRotation() * 0.05, armDegree));
		*/

		// if (joy.getRawButton(5)) {
		// 	armMotorControllerGroup.set(0.01);
		// }
		// if (joy.getRawButton(6)) {
		// 	armMotorControllerGroup.set(-0.01);
		// }
	}
}
