package frc.robot.commands;

import frc.robot.subsystems.SpringClaw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSpringClawDirection extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SpringClaw claw;
	private double desiredClawDirection;

	public SetSpringClawDirection(SpringClaw claw, double desiredClawDirection) {
		this.claw = claw;
		this.desiredClawDirection = desiredClawDirection;
		addRequirements(claw);
	}

	@Override
	public void execute() {
		claw.setSpeed(desiredClawDirection);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
