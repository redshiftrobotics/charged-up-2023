package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExampleSubsystem;

// Sequential command group will run all commands sequentially
// Race command group will run all commands until one finishes
// Parallel command group will run all commands at once and finish when all commands end
// Deadline command group will run all commands until a specific command finishes

public class ExampleCommandGroup extends SequentialCommandGroup {
	public ExampleCommandGroup(ExampleSubsystem subsystem, ExampleSubsystem subsystem2) {
		// if making a deadline group add the following line and remove ExampleCommand from addCommands
		//setDeadline(new ExampleCommand(subsystem))
		addCommands(
				new ExampleCommand(subsystem),
				new ExampleCommand2(subsystem2));
	}
}
