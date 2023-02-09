package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmDegree extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final ArmManager armManager;
	private final double topArmAngle;
	private final double bottomArmAngle;

	/**
	 * Creates a new arm ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public SetArmDegree(ArmManager armManager, double topArmAngle, double bottomArmAngle) {
		this.armManager = armManager;
		this.topArmAngle = topArmAngle;
		this.bottomArmAngle = bottomArmAngle;
		// Using addRequirements() to declare subsystem dependencies
		addRequirements(armManager);
	}

	// Setting the degrees of top arm and bottom arm
	// Called everytime the scheduler runs while the command is scheduled
	@Override
	public void execute() {
		armManager.bottomArm.setDegree(bottomArmAngle);
		armManager.topArm.setDegree(topArmAngle);
	}

	// Called once the command ends or is interrupted
	@Override
	public boolean isFinished() {
		return true;
	}
}
