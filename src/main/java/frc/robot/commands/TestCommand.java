package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class TestCommand extends CommandBase {
	private SwerveDrivetrain drivetrain;
	private ChassisSpeeds speeds;

	public TestCommand(SwerveDrivetrain drivetrain, ChassisSpeeds incomingSpeed) {
		speeds = incomingSpeed;
		this.drivetrain = drivetrain;
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void execute() {
		drivetrain.setSwerveModuleStates(speeds);
	}
}
