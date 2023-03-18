// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.subsystems.SwerveDrivetrain;
// import frc.robot.commands.GoToTagCommand;
// // import frc.robot.subsystems.Camera;

// class FindAndGoToTagCommand extends SequentialCommandGroup {

// 	/**
// 	 * Rotates the robot until the specified tag is in view, then drives to it
// 	 * 
// 	 * @param drivetrain the robot drivetrain
// 	 * @param ID the ID of the target tag
// 	 * @param offset how far from the tag to end
// 	 */
// 	public FindAndGoToTagCommand(SwerveDrivetrain drivetrain, int ID, Translation2d offset) {
// 		// addRequirements(drivetrain);

// 		// addCommands(
// 		// 		new ParallelDeadlineGroup(
// 		// 				// TODO: merge with branch that has camera subsystem and add a boolean function
// 		// 				// 		 that detects if a tag with specified ID is detected
// 		// 				new WaitUntilCommand(Camera::isTagDetected(ID)),
// 		// 				new RotateByCommand(drivetrain, new Rotation2d(10))),

// 		// 		createGoToTagCommand(drivetrain, Camera.getTagTransform(ID), offset));
// 	}
// }