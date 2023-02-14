package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveDriveConstants;

public class DriveDistanceCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final Translation2d driveDistance;
	private Translation2d distance;
	private Translation2d initialPosition;
	private final boolean isFieldRelative;
	private Translation2d position;
	private final PIDController pidController = new PIDController(
			SwerveDriveConstants.ROBOT_VELOCITY_PID_P,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_I,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_D);

	/**
	 * Creates a new DriveDistanceCommand. 
	 * Will drive a distance in x and y directions provided by the caller
	 * @param drivetrain The drivetrain of the robot.
	 * @param driveDistance The distance the robot should move as a Translation2d vector.
	 * @param isFieldRelative Whether the distance the robot should move is oriented around the field or the robot.
	 */
	public DriveDistanceCommand(SwerveDrivetrain drivetrain, Translation2d driveDistance, boolean isFieldRelative) {
		this.drivetrain = drivetrain;
		this.driveDistance = driveDistance;
		initialPosition = drivetrain.getRobotPosition().getTranslation();
		position = initialPosition;
		distance = driveDistance;
		this.isFieldRelative = isFieldRelative;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.setFieldRelative(isFieldRelative);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		position = drivetrain.getRobotPosition().getTranslation();

		// Vector math: A - B =  vector from B to A.
		// First subtract to get the position relative to the initial position.
		// Then subtract that from the target position to get the vector from the position to the target (distance)
		// Results in new vector/Translation2d from the position to the desired position, representing the distance.
		distance = driveDistance.minus(position.minus(initialPosition));

		// Get the new speed as a Translation2d using the output of the pid controller and the angle to the target position.
		Translation2d speed = new Translation2d(pidController.calculate(distance.getNorm()), distance.getAngle());

		// Set the SwerveModuleStates as a new ChassisSpeeds object using the Translation2d above.
		drivetrain.setSwerveModuleStates(new ChassisSpeeds(speed.getX(), speed.getY(), 0));
	}

	// Called once the command ends or is interrupted. Stops the robot.
	@Override
	public void end(boolean interrupted) {
		drivetrain.setSwerveModuleStates(new ChassisSpeeds());
	}

	// Returns true when the robot position is close enough to the desired position
	// and when the speed is slow enough to not drift off target.
	@Override
	public boolean isFinished() {

		if (distance.getNorm() < SwerveDriveConstants.ROBOT_DISTANCE_TOLERANCE
				& drivetrain.getVelocity().getNorm() < SwerveDriveConstants.ROBOT_STOP_VELOCITY_TOLERANCE) {
			return true;
		}
		return false;
	}
}
