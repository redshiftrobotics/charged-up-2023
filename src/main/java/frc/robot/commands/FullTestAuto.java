package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;

public class FullTestAuto extends SequentialCommandGroup {
	public FullTestAuto(SwerveDrivetrain drivetrain) {
		addCommands(
				// new ZeroYaw(drivetrain),
				new DriveDurationCommand(drivetrain, 1.5, 0.045),
				new BalanceCommand(drivetrain));
	}
}
