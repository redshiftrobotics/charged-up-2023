package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends SubsystemBase {

	private final CANSparkMax angularMotor;
	private final CANSparkMax velocityMotor;

	private final CANcoder angularEncoder;
	private final RelativeEncoder velocityEncoder;

	private final PIDController angularPIDController;
	private final PIDController velocityPIDController;

	private SwerveModuleState state = new SwerveModuleState();
	private SwerveModulePosition position = new SwerveModulePosition();

	// Constructor for Swerve Module
	// Makes 2 Motor classes and a PID Controller for one of the Motors
	// Makes an Encoder for the same Motor and sets it up with the motor specifics
	// Parameters: Device Id for Angular Motor, Device Id for Velocity Motor
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

		angularEncoder = new CANcoder(angularEncoderDeviceID);
		velocityEncoder = velocityMotor.getEncoder();

	}

	// Returns the Rotation of the Angular Motor
	// Returns the roations in number of rotations (eg. instead of 2pi or 360 it returns 1)
	// TODO: add modulo operator with the wheel yaw
	public double getYawRotation() {
		return (angularEncoder.getPosition().getValue() * SwerveDriveConstants.ANGULAR_MOTOR_GEAR_RATIO);
	}

	// Returns the velocity of the Velocity motor in meters / second of the wheel
	public double getVelocity() {
		return velocityEncoder.getVelocity() *
				SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO *
				SwerveDriveConstants.WHEEL_CIRCUMFERENCE;
	}

	public SwerveModulePosition getPosition() {
		return position;
	}

	// Sets the speed of the velocity motor and sets the desired state of the angular motor
	// Optimizes the path so that the angular motor doesn't take a longer route than need be
	// Parameters: disered swerve module state
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
	}

}
