package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class SetSlowMode extends CommandBase {
	private final SwerveDrivetrain drivetrain;
	private final boolean mode;

	public SetSlowMode(SwerveDrivetrain drivetrain, boolean mode) {
		this.drivetrain = drivetrain;
		this.mode = mode;
	}

	@Override
	public void execute() {
		drivetrain.setSlowMode(mode);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
