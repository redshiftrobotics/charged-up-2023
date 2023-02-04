package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** A command that drives the robot for a set amount of time. */
public class DriveDurationCommand extends WaitCommand {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final ChassisSpeeds speeds;

	/**
	 * Creates a new DriveDurationCommand.
	 *
	 * @param drivetrain The drivetrain used by this command.
	 * @param seconds The seconds to drive for.
	 * @param speeds The ChassisSpeeds object to drive at.
	 */
	public DriveDurationCommand(SwerveDrivetrain drivetrain, double seconds, ChassisSpeeds speeds) {
		super(seconds);
		this.drivetrain = drivetrain;
		this.speeds = speeds;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	/**
	 * Creates a new DriveDurationCommand.
	 *
	 * @param drivetrain The drivetrain used by this command.
	 * @param seconds The seconds to drive for.
	 * @param forwardSpeed The forward speed to drive at in meters per second.
	 */
	public DriveDurationCommand(SwerveDrivetrain drivetrain, double seconds, double forwardSpeed) {
		this(drivetrain, seconds, new ChassisSpeeds(forwardSpeed, 0, 0));
	}

	// set the chassis speed
	@Override
	public void initialize() {
		super.initialize();
		drivetrain.setFieldRelative(true);
		drivetrain.setSwerveModuleStates(speeds);
	}

	// when command ends set speed to zero
	@Override
	public void end(boolean interrupted) {
		drivetrain.setSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
	}

}
