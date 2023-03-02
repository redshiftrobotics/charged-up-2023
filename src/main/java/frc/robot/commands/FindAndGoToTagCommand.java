import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.SwerveDrivetrain;

class FindAndGoToTagCommand extends SequentialCommandGroup {

	public FindAndGoToTagCommand(SwerveDrivetrain drivetrain, int ID) {
		addRequirements(drivetrain);

		addCommands(
				new ParallelDeadlineGroup(
						Commands.waitUntil(Camera::isTagDetected(ID)),
						null));
	}
}