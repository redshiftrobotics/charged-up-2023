package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

class TestRotateAndDriveAuto extends SequentialCommandGroup {
	public TestRotateAndDriveAuto(SwerveDrivetrain drivetrain) {

		addRequirements(drivetrain);

		addCommands(
				new RotateToCommand(drivetrain, null),
				new DriveDurationCommand(drivetrain, 5, 0.01));
	}

}
