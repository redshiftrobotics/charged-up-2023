package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends SubsystemBase {

	private final CANSparkMax angularMotor;
	private final CANSparkMax velocityMotor;

	private final CANCoder angularEncoder;
	private final CANCoderConfiguration config = new CANCoderConfiguration();

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

		angularPIDController.enableContinuousInput(0, 360);

		velocityPIDController = new PIDController(
				SwerveDriveConstants.VELOCITY_PID_P,
				SwerveDriveConstants.VELOCITY_PID_I,
				SwerveDriveConstants.VELOCITY_PID_D);

		angularMotor = new CANSparkMax(angularMotorDeviceID, MotorType.kBrushless);
		velocityMotor = new CANSparkMax(velocityMotorDeviceID, MotorType.kBrushless);

		config.sensorCoefficient = 1 / 4096;
		config.unitString = "rot";
		config.sensorTimeBase = SensorTimeBase.PerSecond;

		// state.angle = new Rotation2d(Math.PI);

		angularEncoder = new CANCoder(angularEncoderDeviceID, "rio");
		angularEncoder.configAllSettings(config);
		velocityEncoder = velocityMotor.getEncoder();

	}

	// Returns the Rotation of the Angular Motor
	// Returns the roations in number of rotations (eg. instead of 2pi or 360 it returns 1)
	// TODO: add modulo operator with the wheel yaw
	public double getYawRotation() {
		return (angularEncoder.getAbsolutePosition() * SwerveDriveConstants.ANGULAR_MOTOR_GEAR_RATIO);
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
		// state = SwerveModuleState.optimize(desiredState, new Rotation2d(getYawRotation() * 2 * Math.PI));
		state = desiredState;
		// angularPIDController.setSetpoint(state.angle.getRotations());
		// velocityPIDController.setSetpoint(state.speedMetersPerSecond);
	}

	// Calculates the next output value of each PID controller and sets motors to them
	@Override
	public void periodic() {
		double test = angularPIDController.calculate(getYawRotation(), state.angle.getDegrees());
		angularMotor.set(test);
		SmartDashboard.putNumber("Angular PID value", test);

		SmartDashboard.putNumber("state", state.angle.getDegrees());

		// velocityMotor.set(
		// velocityPIDController.calculate(getVelocity()));

		SmartDashboard.putNumber("Angular Encoder", angularEncoder.getAbsolutePosition());
		SmartDashboard.putNumber("Velocity ENcoder", velocityEncoder.getPosition());
	}

}
