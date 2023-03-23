package frc.robot.commands;

import frc.robot.subsystems.JakeClaw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetJakeClawDirection extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final JakeClaw claw;
	private double desiredClawDirection;

	public SetJakeClawDirection(JakeClaw claw, double desiredClawDirection) {
		this.claw = claw;
		this.desiredClawDirection = desiredClawDirection;
		addRequirements(claw);
	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("set jake claw speed", desiredClawDirection);
		claw.setDirection(desiredClawDirection);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
