package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

// The subsystem for controlling the bottom arm
public class BottomArm extends SubsystemBase {

	private final Joystick joy = new Joystick(0);

	// Initializing the motor, encoder, and PID controller
	private final CANSparkMax armMotorOne;
	private final CANSparkMax armMotorTwo;
	private final MotorControllerGroup armMotorControllerGroup;
	private final CANCoder armEncoder;
	private final ProfiledPIDController armPIDController;
	private Rotation2d armAngleRotation2d;
	private Rotation2d minAngleRotation2d;
	private Rotation2d maxAngleRotation2d;

	/*
	private final ArmFeedforward feedForward = new ArmFeedforward(
			ArmConstants.BOTTOM_ARM_PID_S,
			ArmConstants.BOTTOM_ARM_PID_G,
			ArmConstants.BOTTOM_ARM_PID_V,
			ArmConstants.BOTTOM_ARM_PID_A);
	*/

	// Constructor for the bottom arm, which takes in ID's for the motor and encoder as well as for the minimum and maximum degrees
	public BottomArm(int armMotorOneId, int armMotorTwoId, int armEncoderId, double inMinDegree, double inMaxDegree) {
		armPIDController = new ProfiledPIDController(
				ArmConstants.BOTTOM_ARM_PID_P,
				ArmConstants.BOTTOM_ARM_PID_I,
				ArmConstants.BOTTOM_ARM_PID_D,
				new TrapezoidProfile.Constraints(ArmConstants.BOTTOM_ARM_MAX_VELOCITY,
						ArmConstants.BOTTOM_ARM_MAX_ACCEL));
		armMotorOne = new CANSparkMax(armMotorOneId, MotorType.kBrushless);
		armMotorTwo = new CANSparkMax(armMotorTwoId, MotorType.kBrushless);
		armMotorOne.setIdleMode(IdleMode.kBrake);
		armMotorTwo.setIdleMode(IdleMode.kBrake);
		armMotorOne.setInverted(true);

		armMotorControllerGroup = new MotorControllerGroup(armMotorOne, armMotorTwo);
		armEncoder = new CANCoder(armEncoderId);
		minAngleRotation2d = Rotation2d.fromDegrees(inMinDegree);
		maxAngleRotation2d = Rotation2d.fromDegrees(inMaxDegree);
	}

	// Getting the rotation of the encoder
	public Rotation2d getEncoderRotation() {
		return Rotation2d.fromDegrees(armEncoder.getAbsolutePosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO);
	}

	// Setting the desired degree for the bottom arm
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

	// Setting the rotation of the bottom arm
	// Uncomment the bottom comment and comment the top code to slow down the arm for testing
	@Override
	public void periodic() {
		// armMotorControllerGroup.setVoltage(
		// 		armPIDController.calculate(
		// 				getEncoderRotation().getRadians(),
		// 				armAngleRotation2d.getRadians())
		// 				+
		// 				feedForward.calculate(
		// 						armAngleRotation2d.getRadians(),
		// 						armPIDController.getSetpoint().velocity));

		/*
		armMotorControllerGroup.set(armPIDController.calculate(getEncoderRotation() * 0.05, armDegree));
		*/

		// if (joy.getRawButton(5)) {
		// 	armMotorControllerGroup.set(0.5);
		// }
		// if (joy.getRawButton(6)) {
		// 	armMotorControllerGroup.set(-0.5);
		// }

		SmartDashboard.putNumber("Bottom Encoder", armEncoder.getAbsolutePosition());

		// if (joy.getRawButton(9)) {
		// 	armMotorOne.set(0.1);
		// } else if (joy.getRawButton(10)) {
		// 	armMotorOne.set(-0.1);
		// } else {
		// 	armMotorOne.set(0);
		// }
		// if (joy.getRawButton(11)) {
		// 	armMotorTwo.set(0.1);
		// } else if (joy.getRawButton(12)) {
		// 	armMotorTwo.set(-0.1);
		// } else {
		// 	armMotorTwo.set(0);
		// }
	}
}
