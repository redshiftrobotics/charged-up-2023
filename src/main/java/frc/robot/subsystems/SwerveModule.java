package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends PIDSubsystem {

	private final CANSparkMax angularMotor;
	private final CANSparkMax velocityMotor;

	private final AbsoluteEncoder angularMotorEncoder;

	// Constructor for Swerve Module
	// Makes 2 Motor classes and a PID Controller for one of the Motors
	// Makes an Encoder for the same Motor and sets it up with the motor specifics
	// Parameters: Device Id for Angular Motor, Device Id for Velocity Motor
	public SwerveModule(int angularMotorDeviceID, int velocityMotorDeviceID) {
		super(new PIDController(
				SwerveDriveConstants.PID_P,
				SwerveDriveConstants.PID_I,
				SwerveDriveConstants.PID_D));
		angularMotor = new CANSparkMax(angularMotorDeviceID, MotorType.kBrushless);
		velocityMotor = new CANSparkMax(velocityMotorDeviceID, MotorType.kBrushless);

		angularMotorEncoder = angularMotor.getAbsoluteEncoder(Type.kDutyCycle);
		angularMotorEncoder.setPositionConversionFactor(SwerveDriveConstants.ENCODER_POSITION_CONVERSION_FACTOR);
	}

	// Returns the Rotation of the Angular Motor
	// In Radians
	public double getRotation() {
		return angularMotorEncoder.getPosition();
	}

	// Sets the speed of the velocity motor and sets the desired state of the angular motor
	// Optimizes the path so that the angular motor doesn't take a longer route than need be
	// Parameters: disered swerve module state
	public void setSwerveModuleState(SwerveModuleState desiredState) {

		SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getRotation()));

		// https://www.swervedrivespecialties.com/products/mk4-swerve-module
		velocityMotor.set(state.speedMetersPerSecond / SwerveDriveConstants.VELOCITY_MOTOR_MAX_SPEED_METRIC);

		SwerveModule.super.m_controller.setSetpoint(state.angle.getRadians());

	}

	@Override
	protected double getMeasurement() {
		return getRotation();
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		angularMotor.set(output);
	}

}
