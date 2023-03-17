package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends SubsystemBase {

	private final CANSparkMax angularMotor;
	private final CANSparkMax velocityMotor;

	private final CANCoder angularEncoder;
	// Configuration settings for angularEncoder
	private final CANCoderConfiguration config = new CANCoderConfiguration();

	private final RelativeEncoder velocityEncoder;

	private final PIDController angularPIDController;
	// private final PIDController velocityPIDController;

	// private final SparkMaxPIDController angularSparkMaxPIDController;
	private final SparkMaxPIDController velocitySparkMaxPIDController;

	private SwerveModuleState state = new SwerveModuleState();
	private SwerveModulePosition position = new SwerveModulePosition();

	private double angularZero = 0;

	private String smartdashboardID;

	private String testString;

	// private double velocityMotorSpeed = 0;

	// private double velocityMotorSpeed = 0;
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
			int angularEncoderDeviceID,
			double rotationZero) {
		testString = "" + angularEncoderDeviceID;
		// this.smartdashboardID = dashboardID;
		angularZero = rotationZero;
		angularPIDController = new PIDController(
				SwerveDriveConstants.ANGULAR_PID_P,
				SwerveDriveConstants.ANGULAR_PID_I,
				SwerveDriveConstants.ANGULAR_PID_D);

		angularPIDController.enableContinuousInput(0, 360);

		angularMotor = new CANSparkMax(angularMotorDeviceID, MotorType.kBrushless);
		velocityMotor = new CANSparkMax(velocityMotorDeviceID, MotorType.kBrushless);

		// angularSparkMaxPIDController = angularMotor.getPIDController();
		velocitySparkMaxPIDController = velocityMotor.getPIDController();

		velocitySparkMaxPIDController.setP(SwerveDriveConstants.VELOCITY_PID_P);
		velocitySparkMaxPIDController.setI(SwerveDriveConstants.VELOCITY_PID_I);
		velocitySparkMaxPIDController.setD(SwerveDriveConstants.VELOCITY_PID_D);
		velocitySparkMaxPIDController.setFF(SwerveDriveConstants.VELOCITY_PID_FF);
		velocitySparkMaxPIDController.setOutputRange(-1, 1);
		velocitySparkMaxPIDController.setIZone(0);

		// Rotational units: 4096 / rotation
		config.sensorCoefficient = 1 / 4096;
		// Using rotations from encoder
		config.unitString = "rot";
		// Encoder uses seconds for time unit - used for angular velocity
		config.sensorTimeBase = SensorTimeBase.PerSecond;

		SmartDashboard.putNumber("RPM", 0);

		// state.angle = new Rotation2d(Math.PI);

		angularEncoder = new CANCoder(angularEncoderDeviceID, "rio");
		angularEncoder.configAllSettings(config);
		velocityEncoder = velocityMotor.getEncoder();
		velocityEncoder.setPositionConversionFactor(
				SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO *
						SwerveDriveConstants.WHEEL_CIRCUMFERENCE);
		velocityEncoder.setVelocityConversionFactor(
				SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO *
						SwerveDriveConstants.WHEEL_CIRCUMFERENCE);

	}

	/** Returns the angle of the swerve module
	 * @return The rotations in number of rotations - for a full rotation, returns 1 instead of 2pi or 360
	 */
	public double getYawRotation() {
		return (angularEncoder.getAbsolutePosition() + angularZero);
	}

	/** Returns the current velocity of the swerve module
	 * @return rotation of the wheel in meters/second
	 */
	public double getVelocity() {
		// Convert from RPM to MPS
		return (velocityEncoder.getVelocity() / 60) * SwerveDriveConstants.WHEEL_CIRCUMFERENCE;
	}

	/** Gets the position of the swerve module as a SwerveModulePosition 
	 * @return SwerveModulePosition from velocity encoder position and angular encoder position
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(velocityEncoder.getPosition(),
				new Rotation2d(angularEncoder.getAbsolutePosition()));
	}

	/** Sets the speed and desired angle of the module
	 * Optimizes the rotation of the angular motor to shortest path to a desired angle
	 * @param desiredState derired state of the SwerveModule
	 */
	public void setState(SwerveModuleState desiredState) {
		// state = SwerveModuleState.optimize(desiredState, new Rotation2d(getYawRotation() * 2 * Math.PI));
		state = desiredState;
		// angularPIDController.setSetpoint(state.angle.getRotations());
		// velocityPIDController.setSetpoint(state.speedMetersPerSecond);
	}

	// Calculates the next output value of each PID controller and sets motors to them
	@Override
	public void periodic() {
		// double test = angularPIDController.calculate(getYawRotation(), state.angle.getDegrees());
		// angularMotor.set(test);
		// SmartDashboard.putNumber("Angular PID value", test);

		// // Convert from RPM to MPS
		// velocitySparkMaxPIDController.setReference(
		// 		(state.speedMetersPerSecond * 60) / SwerveDriveConstants.WHEEL_CIRCUMFERENCE,
		// 		CANSparkMax.ControlType.kVelocity);
		// SmartDashboard.putNumber("Velocity set value",
		// 		(state.speedMetersPerSecond * 60) / SwerveDriveConstants.WHEEL_CIRCUMFERENCE);
		// SmartDashboard.putNumber(testString, angularEncoder.getAbsolutePosition());
	}

	// stops motor and ends PID
	public void stop() {
		velocityMotor.stopMotor();
		angularMotor.stopMotor();
		// velocityPIDController.setSetpoint(0);
	}
}
