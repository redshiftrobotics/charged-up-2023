package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawGrabCommand extends CommandBase {
	private final Claw claw;

	public ClawGrabCommand(Claw claw) {
		this.claw = claw;
	}

	@Override
	public void initialize() {
		claw.grabItem();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	//@Override
	//public void execute() {

	//}
}
