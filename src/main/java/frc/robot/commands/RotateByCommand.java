package frc.robot.commands;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that rotates the robot a given amount in place. */
public class RotateByCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final Rotation2d desiredRotation;
	private final Rotation2d initialRotation;
	private Rotation2d rotation = new Rotation2d();
	private final PIDController pidController = new PIDController(
			SwerveDriveConstants.ROBOT_ANGULAR_PID_P,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_I,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_D);

	/**
	 * Creates a new RotateByCommand.
	 * Rotate the robot by a given amount in place.
	 * 
	 * @param drivetrain The robot's drivetrain used by this command.
	 * @param desiredRotation How much the robot should rotate by.
	 */
	public RotateByCommand(SwerveDrivetrain drivetrain, Rotation2d desiredRotation) {
		this.drivetrain = drivetrain;
		this.desiredRotation = desiredRotation;
		initialRotation = drivetrain.getRobotPosition().getRotation();

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Set fieldRelative to false to begin.
	@Override
	public void initialize() {
		drivetrain.setFieldRelative(false);
	}

	// Called every time the scheduler runs while the command is scheduled.
	// Get current rotation, provide it to the pidController, and set the new speed.
	@Override
	public void execute() {
		rotation = drivetrain.getRobotPosition().getRotation().minus(initialRotation);
		double rotationSpeed = pidController.calculate(rotation.getRadians() - desiredRotation.getRadians());
		drivetrain.setSwerveModuleStates(new ChassisSpeeds(0, 0, rotationSpeed));
	}

	// Stop robot when command ends.
	@Override
	public void end(boolean interrupted) {
		drivetrain.setSwerveModuleStates(new ChassisSpeeds());
	}

	// Returns true when the error is low enough and the speed is slow enough to stop.
	@Override
	public boolean isFinished() {
		if (pidController.getPositionError() < SwerveDriveConstants.ROBOT_ANGLE_TOLERANCE &
				drivetrain.getRotationSpeed() < SwerveDriveConstants.ROBOT_STOP_ROTATION_TOLERANCE) {
			return true;
		}
		return false;
	}
}
