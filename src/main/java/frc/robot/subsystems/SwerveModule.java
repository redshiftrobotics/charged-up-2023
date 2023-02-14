package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends SubsystemBase {

	private final CANSparkMax angularMotor;
	private final CANSparkMax velocityMotor;

	private final CANCoder angularEncoder;
	private final RelativeEncoder velocityEncoder;

	private final PIDController angularPIDController;
	private final PIDController velocityPIDController;

	private SwerveModuleState state = new SwerveModuleState();
	private SwerveModulePosition position = new SwerveModulePosition();

	/** Constructor for Swerve Module
	 * Makes 2 Motor classes and a PID Controller for one of the Motors.
	 * Makes an Encoder for the same Motor and sets it up with the motor specifics.
	 * @param angularMotorDeviceID Device ID for angular encoder
	 * @param velocityMotorDeviceID Device ID for velocity motor
	 * @param angularEncoderDeviceID Device ID for the angular encoder
	*/
	public SwerveModule(
			int angularMotorDeviceID,
			int velocityMotorDeviceID,
			int angularEncoderDeviceID) {

		angularPIDController = new PIDController(
				SwerveDriveConstants.ANGULAR_PID_P,
				SwerveDriveConstants.ANGULAR_PID_I,
				SwerveDriveConstants.ANGULAR_PID_D);

		velocityPIDController = new PIDController(
				SwerveDriveConstants.VELOCITY_PID_P,
				SwerveDriveConstants.VELOCITY_PID_I,
				SwerveDriveConstants.VELOCITY_PID_D);

		angularMotor = new CANSparkMax(angularMotorDeviceID, MotorType.kBrushless);
		velocityMotor = new CANSparkMax(velocityMotorDeviceID, MotorType.kBrushless);

		angularEncoder = new CANCoder(angularEncoderDeviceID, "rio");
		velocityEncoder = velocityMotor.getEncoder();
		velocityEncoder.setPositionConversionFactor(
				SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO *
						SwerveDriveConstants.WHEEL_CIRCUMFERENCE);
		velocityEncoder.setVelocityConversionFactor(
				SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO *
						SwerveDriveConstants.WHEEL_CIRCUMFERENCE);

	}

	// TODO: add modulo operator with the wheel yaw
	/** Returns the angle of the swerve module
	 * @return The rotations in number of rotations - for a full rotation, returns 1 instead of 2pi or 360
	 */
	public double getYawRotation() {
		return (angularEncoder.getAbsolutePosition() * SwerveDriveConstants.ANGULAR_ENCODER_GEAR_RATIO);
	}

	/** Returns the current velocity of the swerve module
	 * @return rotation of the wheel in meters/second
	 */
	public double getVelocity() {
		return velocityEncoder.getVelocity();
	}

	/** Gets the position of the swerve module as a SwerveModulePosition 
	 * @return SwerveModulePosition from velocity encoder position and angular encoder position
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(velocityEncoder.getPosition(),
				new Rotation2d(angularEncoder.getAbsolutePosition() * SwerveDriveConstants.ANGULAR_ENCODER_GEAR_RATIO));
	}

	/** Sets the speed and desired angle of the module
	 * Optimizes the rotation of the angular motor to shortest path to a desired angle
	 * @param desiredState derired state of the SwerveModule
	 */
	public void setState(SwerveModuleState desiredState) {
		state = SwerveModuleState.optimize(desiredState, new Rotation2d(getYawRotation() * 2 * Math.PI));
		angularPIDController.setSetpoint(state.angle.getRotations());
		velocityPIDController.setSetpoint(state.speedMetersPerSecond);
	}

	// Calculates the next output value of each PID controller and sets motors to them
	@Override
	public void periodic() {
		angularMotor.set(
				angularPIDController.calculate(getYawRotation()));

		velocityMotor.set(
				velocityPIDController.calculate(getVelocity()));

		SmartDashboard.putNumber("Angular Encoder", angularEncoder.getPosition());
		SmartDashboard.putNumber("Velocity ENcoder", velocityEncoder.getPosition());
	}

	// stops motor and ends PID
	public void stop() {
		velocityMotor.stopMotor();
		velocityPIDController.setSetpoint(0);
	}

}
