// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.subsystems.ExampleSubsystem;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /* Example command group constructed from other commands and
//  * command groups. Some are made inline and some are from other files.
//  * Make sure you understand this as it will be extremely useful in
//  * composing complex commands and systems
// */
// public class ExampleRecursiveCommandGroup extends ParallelCommandGroup {
// 	public ExampleRecursiveCommandGroup(ExampleSubsystem subsystem, ExampleSubsystem subsystem2,
// 			Arm arm, Drivetrain drivetrain, Encoder Encoder) {
// 		addCommands(
// 				new ExampleCommand(subsystem),
// 				new SequentialCommandGroup(
// 						new ExampleCommand(subsystem),
// 						new WaitCommand(1),
// 						new ParallelRaceGroup(
// 								new RaiseArmCommand(arm),
// 								new MoveFowardCommand(drivetrain))),
// 				new ExampleCommandGroup(subsystem, subsystem2));
// 	}
// }
