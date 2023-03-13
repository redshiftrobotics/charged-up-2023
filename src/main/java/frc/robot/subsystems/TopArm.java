package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// The subsystem for controlling the top arm
public class TopArm extends SubsystemBase {
	// Initializing the motor, encoder, and PID controller

	private final Joystick joy = new Joystick(0);

	private final CANSparkMax armMotor;
	private final CANCoder armEncoder;
	private final ProfiledPIDController armPIDController;
	private Rotation2d armAngleRotation2d;
	private Rotation2d minAngleRotation2d;
	private Rotation2d maxAngleRotation2d;

	/* 
	private final ArmFeedforward feedForward = new ArmFeedforward(
			ArmConstants.TOP_ARM_PID_S,
			ArmConstants.TOP_ARM_PID_G,
			ArmConstants.TOP_ARM_PID_V,
			ArmConstants.TOP_ARM_PID_A); 
	*/

	// Constructor for the top arm, which takes in ID's for the motor and encoder as well as for the minimum and maximum degrees
	public TopArm(int armMotorId, int armEncoderId, double inMinDegree, double inMaxDegree) {

		armPIDController = new ProfiledPIDController(
				ArmConstants.TOP_ARM_PID_P,
				ArmConstants.TOP_ARM_PID_I,
				ArmConstants.TOP_ARM_PID_D,
				new TrapezoidProfile.Constraints(ArmConstants.TOP_ARM_MAX_VELOCITY, ArmConstants.TOP_ARM_MAX_ACCEL));

		armMotor = new CANSparkMax(armMotorId, MotorType.kBrushless);
		armMotor.setIdleMode(IdleMode.kBrake);
		armEncoder = new CANCoder(armEncoderId);
		minAngleRotation2d = Rotation2d.fromDegrees(inMinDegree);
		maxAngleRotation2d = Rotation2d.fromDegrees(inMaxDegree);
		armAngleRotation2d = Rotation2d.fromDegrees(100);
	}

	// Getting the rotation of the encoder
	public Rotation2d getEncoderRotation() {
		return Rotation2d.fromDegrees(armEncoder.getAbsolutePosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO);
	}

	// Setting the desired degree for the top arm
	public void setDegree(double desiredDegree) {
		// Limiting the angle the arm can be set to to between the minimum and maximum degrees
		// Minimum degree will not always be set to zero. 
		if (minAngleRotation2d.getDegrees() + desiredDegree > maxAngleRotation2d.getDegrees()) {
			armAngleRotation2d = maxAngleRotation2d;
		}
		if (minAngleRotation2d.getDegrees() + desiredDegree < minAngleRotation2d.getDegrees()) {
			armAngleRotation2d = minAngleRotation2d;
		} else {
			armAngleRotation2d = Rotation2d.fromDegrees(minAngleRotation2d.getDegrees() + desiredDegree);
		}
	}

	// Setting the rotation of the top arm
	// Uncomment the bottom comment and comment the top code to slow down the arm for testing
	@Override
	public void periodic() {
		armMotor.setVoltage(
				armPIDController.calculate(
						getEncoderRotation().getRadians(),
						armAngleRotation2d.getRadians())
		// +
		// feedForward.calculate(
		// 		armAngleRotation2d.getRadians(),
		// 		armPIDController.getSetpoint().velocity)
		);

		// armMotor.set(armPIDController.calculate(getEncoderRotation() * 0.05, armDegree));

		// if (joy.getRawButton(3)) {
		// 	armMotor.set(0.1);
		// } else if (joy.getRawButton(4)) {
		// 	armMotor.set(-0.1);
		// } else {
		// 	armMotor.set(0);
		// }
		SmartDashboard.putNumber("Top Encoder", armEncoder.getAbsolutePosition());
	}
}
