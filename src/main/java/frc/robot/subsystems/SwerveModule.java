package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends SubsystemBase {

	private final CANSparkMax angularMotor;
	private final CANSparkMax velocityMotor;

	private final CANCoder angularEncoder;
	private final CANCoderConfiguration config = new CANCoderConfiguration();

	private final RelativeEncoder velocityEncoder;

	private final PIDController angularPIDController;
	// private final PIDController velocityPIDController;

	// private final SparkMaxPIDController angularSparkMaxPIDController;
	private final SparkMaxPIDController velocitySparkMaxPIDController;

	private SwerveModuleState state = new SwerveModuleState();
	private SwerveModulePosition position = new SwerveModulePosition();

	// private double velocityMotorSpeed = 0;

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

		// velocityPIDController = new PIDController(
		// SwerveDriveConstants.VELOCITY_PID_P,
		// SwerveDriveConstants.VELOCITY_PID_I,
		// SwerveDriveConstants.VELOCITY_PID_D);

		angularMotor = new CANSparkMax(angularMotorDeviceID, MotorType.kBrushless);
		velocityMotor = new CANSparkMax(velocityMotorDeviceID, MotorType.kBrushless);

		// angularSparkMaxPIDController = angularMotor.getPIDController();
		velocitySparkMaxPIDController = velocityMotor.getPIDController();

		velocitySparkMaxPIDController.setP(SwerveDriveConstants.VELOCITY_PID_P);
		velocitySparkMaxPIDController.setI(SwerveDriveConstants.VELOCITY_PID_I);
		velocitySparkMaxPIDController.setD(SwerveDriveConstants.VELOCITY_PID_D);
		velocitySparkMaxPIDController.setFF(0.00000);
		velocitySparkMaxPIDController.setOutputRange(-1, 1);
		velocitySparkMaxPIDController.setIZone(0);

		SmartDashboard.putNumber("Velocity P", SwerveDriveConstants.ANGULAR_PID_P);
		SmartDashboard.putNumber("Velocity I", SwerveDriveConstants.ANGULAR_PID_I);
		SmartDashboard.putNumber("Velocity D", SwerveDriveConstants.ANGULAR_PID_D);
		SmartDashboard.putNumber("Velocity FF", 0);

		config.sensorCoefficient = 1 / 4096;
		config.unitString = "rot";
		config.sensorTimeBase = SensorTimeBase.PerSecond;

		SmartDashboard.putNumber("RPM", 0);

		// state.angle = new Rotation2d(Math.PI);

		angularEncoder = new CANCoder(angularEncoderDeviceID, "rio");
		angularEncoder.configAllSettings(config);
		velocityEncoder = velocityMotor.getEncoder();

		velocitySparkMaxPIDController.setFeedbackDevice(velocityEncoder);

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
				SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO /**
																SwerveDriveConstants.WHEEL_CIRCUMFERENCE*/
		;
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

		// SmartDashboard.putNumber("state", state.angle.getDegrees());

		// double velocityTest = velocityPIDController.calculate(getVelocity(), state.speedMetersPerSecond);

		// SmartDashboard.putNumber("Velocity PID output", velocityTest);

		// velocityMotorSpeed += velocityTest * Constants.periodicFrequency;

		// SmartDashboard.putNumber("Velocity Motor speed", velocityMotorSpeed);

		// makes sure speed is under 1 and over -1

		// if (velocityMotorSpeed > 1) {
		// 	velocityMotorSpeed = 1;
		// } else if (velocityMotorSpeed < -1) {
		// 	velocityMotorSpeed = -1;
		// }

		// SmartDashboard.putNumber("Velocity Motor speed", velocityMotorSpeed);

		// velocityMotor.set(velocityMotorSpeed);

		// SmartDashboard.putNumber("Angular Encoder absolute", angularEncoder.getAbsolutePosition());

		// SmartDashboard.putNumber("Angular Encoder", angularEncoder.getPosition());

		// SmartDashboard.putNumber("Velocity Encoder Velocity", velocityEncoder.getVelocity());

		// SmartDashboard.putNumber("Velocity Encoder", velocityEncoder.getPosition());

		double velocityTest = (state.speedMetersPerSecond / SwerveDriveConstants.WHEEL_CIRCUMFERENCE)
				/ SwerveDriveConstants.VELOCITY_MOTOR_GEAR_RATIO;

		SmartDashboard.putNumber("Velocity test", velocityTest);

		double p = SmartDashboard.getNumber("Velocity P", 0);
		double i = SmartDashboard.getNumber("Velocity I", 0);
		double d = SmartDashboard.getNumber("Velocity D", 0);

		Joystick joy = new Joystick(0);
		velocitySparkMaxPIDController.setP(p);
		velocitySparkMaxPIDController.setI(i);
		velocitySparkMaxPIDController.setD(d);

		double ff = (joy.getThrottle() + 1) / 20;

		velocitySparkMaxPIDController.setFF(ff);

		SmartDashboard.putNumber("Velocity FF", ff);

		// velocitySparkMaxPIDController.setFF(SmartDashboard.getNumber("Velocity FF", 0));

		velocitySparkMaxPIDController.setReference(SmartDashboard.getNumber("RPM", 0),
				CANSparkMax.ControlType.kVelocity);

		// velocityMotor.set(();
	}
}
