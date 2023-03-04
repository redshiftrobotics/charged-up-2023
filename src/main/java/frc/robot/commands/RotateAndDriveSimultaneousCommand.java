// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateAndDriveSimultaneousCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final Translation2d initialPosition;
	private Translation2d position;
	private Rotation2d rotation;
	private Translation2d distance;
	private final Translation2d driveDistance;
	private final Rotation2d desiredRotation;
	private final PIDController pidRotation = new PIDController(
			SwerveDriveConstants.ROBOT_ANGULAR_PID_P,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_I,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_D);
	private final PIDController pidVelocity = new PIDController(
			SwerveDriveConstants.ROBOT_VELOCITY_PID_P,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_I,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_D);

	/**
	 * Moves and rotates the robot a given amount.
	 * @param drivetrain The drivetrain of the robot.
	 * @param desiredRotation The angle to rotate to (ccw).
	 * @param fieldRelativeRotation If the angle is field relative or not.
	 * @param driveDistance The amount to drive by in meters.
	 * @param fieldRelativeDistance If the driveDistance is field relative or not.
	 */
	public RotateAndDriveSimultaneousCommand(SwerveDrivetrain drivetrain,
			Rotation2d desiredRotation, boolean fieldRelativeRotation,
			Translation2d driveDistance, boolean fieldRelativeDistance) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);

		initialPosition = drivetrain.getRobotPosition().getTranslation();
		position = initialPosition;
		rotation = drivetrain.getRobotPosition().getRotation();
		// Uses field-relative drive by default; modifies driveDistance accordingly.
		// If given is not field-relative, rotate by the rotation to make it field-relative.
		this.driveDistance = (fieldRelativeDistance) ? driveDistance : driveDistance.rotateBy(rotation);
		// Uses field-relative rotation by default; modifies desired rotation accordingly.
		// If given is not field-relative, rotate by the rotation to make it field-relative.
		this.desiredRotation = (fieldRelativeRotation) ? desiredRotation : desiredRotation.plus(rotation);

		pidRotation.setTolerance(SwerveDriveConstants.ROBOT_ANGLE_TOLERANCE,
				SwerveDriveConstants.ROBOT_STOP_ROTATION_TOLERANCE);
		pidVelocity.setTolerance(SwerveDriveConstants.ROBOT_DISTANCE_TOLERANCE,
				SwerveDriveConstants.ROBOT_STOP_VELOCITY_TOLERANCE);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.setFieldRelative(true);
		rotation = drivetrain.getRobotPosition().getRotation();
		// Do one calculation so the pidControllers have error values for execute()
		pidRotation.calculate(
				((rotation.getRadians() - desiredRotation.getRadians() + Math.PI) % Math.PI * 2) - Math.PI);
		pidVelocity.calculate(driveDistance.getNorm());
	}

	// Called every time the scheduler runs while the command is scheduled.
	// Get current rotation and distance, provide it to the pidControllers, and set the new speed.
	@Override
	public void execute() {
		rotation = drivetrain.getRobotPosition().getRotation();
		double rotationSpeed = 0;

		// If robot is not already at the correct rotation, set the new rotation speed.
		if (!pidRotation.atSetpoint()) {
			// Get the difference in position and alter it to make the PIDController take the shortest path.
			// https://www.desmos.com/calculator/40g32n3myv
			rotationSpeed = pidRotation.calculate(
					((rotation.getRadians() - desiredRotation.getRadians() + Math.PI) % Math.PI * 2) - Math.PI);
		}

		position = drivetrain.getRobotPosition().getTranslation();
		distance = driveDistance.minus(position.minus(initialPosition));
		Translation2d velocity = new Translation2d();

		// If robot is not already at the correct position, set the new velocity.
		if (!pidVelocity.atSetpoint()) {
			velocity = new Translation2d(pidVelocity.calculate(distance.getNorm()), rotation);
		}

		drivetrain.setSwerveModuleStates(new ChassisSpeeds(velocity.getX(), velocity.getY(), rotationSpeed));
	}

	// Called once the command ends or is interrupted.
	// Reset the speeds to 0.
	@Override
	public void end(boolean interrupted) {
		drivetrain.setSwerveModuleStates(new ChassisSpeeds());
	}

	// Returns true when the robot is in the right position and rotation.
	@Override
	public boolean isFinished() {
		return pidRotation.atSetpoint() & pidVelocity.atSetpoint();
	}
}
