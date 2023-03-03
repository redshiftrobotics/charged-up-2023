package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

class TestAuto extends SequentialCommandGroup {
	public TestAuto(SwerveDrivetrain drivetrain) {

		addRequirements(drivetrain);

		// addCommands(
		// 		new DriveDurationCommand(drivetrain, 1, SwerveDriveConstants.MAX_SPEED),
		// 		new DriveDurationCommand(drivetrain, 1, -Constants.MAX_SPEED));
	}
}