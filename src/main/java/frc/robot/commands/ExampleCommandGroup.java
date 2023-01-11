package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Sequential command group will run all commands sequentially
// ParallelCommandGroup will run all commands in parallel and finish when all commands are finished
// ParallelRaceGroup will run all commands until one finishes, and interupt the rest
// ParallelDeadlineGroup will run all commands until a specified "deadline" command finishes, then interupt the rest

public class ExampleCommandGroup extends SequentialCommandGroup {
	public ExampleCommandGroup(ExampleSubsystem subsystem, ExampleSubsystem subsystem2) {
		// if making a deadline group add the following line and remove ExampleCommand from addCommands
		//setDeadline(new ExampleCommand(subsystem))
		addCommands(
				new ExampleCommand(subsystem),
				new WaitCommand(1.0),
				new ExampleCommand(subsystem2));
	}
}
