package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that instantly stops the robot */
public class StopCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;

	/**
	 * Creates a new StopCommand 
	 *
	 * @param drivetrain the drivetrain to stop
	 */
	public StopCommand(SwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// stops drivetrain
	@Override
	public void initialize() {
		drivetrain.stop();
	}

	public boolean isFinished() {
		return true;
	}
}
