package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmDegree extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Arm bottomArm;
	private final Arm topArm;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public SetArmDegree(Arm bottomArm, Arm topArm, double desiredDegree) {
		this.bottomArm = bottomArm;
		this.topArm = topArm;

		addRequirements(bottomArm);
		addRequirements(topArm);
	}

	@Override
	public void initialize() {
		bottomArm.setDegree(0);
		topArm.setDegree(0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
