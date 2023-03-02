package frc.robot.commands;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetClawDirection extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Claw claw;
	private double desiredClawDirection;

	public SetClawDirection(Claw claw, double desiredClawDirection) {
		this.claw = claw;
		this.desiredClawDirection = desiredClawDirection;
		addRequirements(claw);
	}

	@Override
	public void execute() {
		claw.setDirection(desiredClawDirection);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
