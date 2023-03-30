package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class ZeroYaw extends CommandBase {
	private final SwerveDrivetrain drivetrain;

	public ZeroYaw(SwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.zeroYaw();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
