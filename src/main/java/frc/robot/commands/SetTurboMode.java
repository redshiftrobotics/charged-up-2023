package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class SetTurboMode extends CommandBase {
	private final SwerveDrivetrain drive;
	private final boolean mode;

	public SetTurboMode(SwerveDrivetrain drive, boolean mode) {
		this.drive = drive;
		this.mode = mode;
		addRequirements(drive);
	}

	@Override
	public void execute() {
		drive.setTurboMode(mode);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
