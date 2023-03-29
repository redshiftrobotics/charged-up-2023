package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

// The subsystem for controlling the bottom arm and motors associated with it
public class BottomArm extends SubsystemBase {

	private final Joystick joy = new Joystick(1);

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

	/* Constructor for the bottom arm, which takes in ID's for the motor and encoder as well as for the minimum and maximum
	 degrees
	*/
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
		armMotorOne.setInverted(false);

		armMotorControllerGroup = new MotorControllerGroup(armMotorOne, armMotorTwo);
		armEncoder = new CANCoder(armEncoderId);
		armPIDController.enableContinuousInput(0, 2 * Math.PI);
		minAngleRotation2d = Rotation2d.fromDegrees(inMinDegree);
		maxAngleRotation2d = Rotation2d.fromDegrees(inMaxDegree);
		armAngleRotation2d = Rotation2d.fromDegrees(ArmConstants.BOTTOM_ARM_START_DEGREE);
	}

	public Rotation2d getTarget() {
		return armAngleRotation2d;
	}

	// Getting the rotation of the encoder which will be used for moving the arm and setting limits for the arm
	public Rotation2d getEncoderRotation() {
		return Rotation2d.fromDegrees(armEncoder.getAbsolutePosition() * ArmConstants.ARM_MOTOR_GEAR_RATIO);
	}

	// Setting the desired degree for the bottom arm (only used for when/ if max degree is set to 0)
	public void setDegree(double desiredDegree) {
		// Limiting the angle the arm can be set to to between the minimum and maximum degrees
		// Minimum degree will not always be set to zero. 
		if (maxAngleRotation2d.getDegrees() < minAngleRotation2d.getDegrees()) {
			if (desiredDegree < maxAngleRotation2d.getDegrees() || desiredDegree > minAngleRotation2d.getDegrees()) {
				armAngleRotation2d = Rotation2d.fromDegrees(desiredDegree);
			}
		} else {
			if (desiredDegree > minAngleRotation2d.getDegrees() && desiredDegree < maxAngleRotation2d.getDegrees()) {
				armAngleRotation2d = Rotation2d.fromDegrees(desiredDegree);
			}
		}

	}

	// Setting the rotation of the bottom arm
	//           >>> Uncomment the bottom comment and comment the top code to slow down the arm for testing <<<
	@Override
	public void periodic() {
		// armMotorControllerGroup.setVoltage(
		// 		armPIDController.calculate(
		// 				getEncoderRotation().getRadians(),
		// 				armAngleRotation2d.getRadians())
		// 				+
		// 				feedForward.calculate(
		// 						armAngleRotation2d.getRadians(),
		// 						armPIDController.getSetpoint().velocity)
		// );

		/*
		armMotorControllerGroup.set(armPIDController.calculate(getEncoderRotation() * 0.05, armDegree));
		*/

		// if (joy.getRawButton(5)) {
		// 	armMotorControllerGroup.set(0.05);
		// } else if (joy.getRawButton(6)) {
		// 	armMotorControllerGroup.set(-0.05);
		// } else {
		// 	armMotorControllerGroup.set(0);
		// }

		SmartDashboard.putNumber("Bottom Encoder", armEncoder.getAbsolutePosition());

		SmartDashboard.putNumber("Bottom Set Value", armAngleRotation2d.getDegrees());

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

		if (joy.getRawButton(4)) {
			armMotorControllerGroup.set(.3);
		} else if (joy.getRawButton(6)) {
			armMotorControllerGroup.set(-.3);
		} else {
			armMotorControllerGroup.set(0);
		}
	}
}
// TomarTopia CC 2023, the dark souls of papercuts