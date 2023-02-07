package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

	// Constructor for Swerve Module
	// Makes 2 Motor classes and a PID Controller for one of the Motors
	// Makes an Encoder for the same Motor and sets it up with the motor specifics
	// Parameters: Device Id for Angular Motor, Device Id for Velocity Motor
	public SwerveModule(
			int angularMotorDeviceID,
			int velocityMotorDeviceID,
			int angularEncoderDeviceID) {
	}

	// Returns the Rotation of the Angular Motor
	// Returns the roations in number of rotations (eg. instead of 2pi or 360 it returns 1)
	// TODO: add modulo operator with the wheel yaw
	public double getYawRotation() {
		return 0;
	}

	// Returns the velocity of the Velocity motor in meters / second of the wheel
	public double getVelocity() {
		return 0;
	}

	// Sets the speed of the velocity motor and sets the desired state of the angular motor
	// Optimizes the path so that the angular motor doesn't take a longer route than need be
	// Parameters: disered swerve module state
	public void setSwerveModuleState(SwerveModuleState desiredState) {
	}

	// Calculates the next output value of each PID controller and sets motors to them
	@Override
	public void periodic() {
	}

}
