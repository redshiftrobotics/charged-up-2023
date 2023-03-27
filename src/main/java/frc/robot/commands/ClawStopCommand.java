package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawStopCommand extends CommandBase {
	private final Claw claw;

	public ClawStopCommand(Claw claw) {
		this.claw = claw;
	}

	@Override
	public void initialize() {
		claw.stop();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	//@Override
	//public void execute() {

	//}
}
