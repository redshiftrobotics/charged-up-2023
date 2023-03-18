package frc.robot.commands;

import frc.robot.subsystems.ArmManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmDegreeCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final ArmManager armManager;
	private final double desiredBottomArmDegree;
	private final double desiredTopArmDegree;

	/**
	 * Creates a new arm ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public SetArmDegreeCommand(ArmManager armManager, double desiredBottomArmDegree, double desiredTopArmDegree) {
		this.armManager = armManager;
		this.desiredBottomArmDegree = desiredBottomArmDegree;
		this.desiredTopArmDegree = desiredTopArmDegree;
		// Using addRequirements() to declare subsystem dependencies
		addRequirements(armManager);
	}

	// Setting the degrees of top arm and bottom arm
	// Called every time the scheduler runs while the command is scheduled
	@Override
	public void execute() {
		armManager.setBothArmDegree(desiredBottomArmDegree, desiredTopArmDegree);
	}

	// Called once the command ends or is interrupted
	@Override
	public boolean isFinished() {
		return true;
	}
}
