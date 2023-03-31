package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;

public class FullTestAuto extends SequentialCommandGroup {
	public FullTestAuto(SwerveDrivetrain drivetrain) {
		addCommands(
				// new ZeroYaw(drivetrain),
				new DriveDurationCommand(drivetrain, 3, 0.02),
				new BalanceCommand(drivetrain));
	}
}
