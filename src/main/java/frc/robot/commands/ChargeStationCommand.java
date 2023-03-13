package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * SequentialCommandGroup that first drives onto the charge station, then tries to balance on it.
 */
public class ChargeStationCommand extends SequentialCommandGroup {
	/**
	 * Create a CommandGroup to automatically move the drivetrain onto the charge station, then balance on it.
	 * @param drivetrain (the drivetrain of the robot.)
	 */
	public ChargeStationCommand(SwerveDrivetrain drivetrain) {
		addCommands(
				new DriveToRampCommand(drivetrain),
				new BalanceCommand(drivetrain));
	}
}
