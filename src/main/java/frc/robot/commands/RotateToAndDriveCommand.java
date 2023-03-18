package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Rotate the robot to a given rotation relative to the field and drive forward a given distance in meters.
 */
public class RotateToAndDriveCommand extends SequentialCommandGroup {
	/** Rotate the robot to a given rotation relative to the field and drive forward a given distance in meters.
	 * 
	 * @param drivetrain The drivetrain of the robot.
	 * @param angle The field angle to rotate to as a Rotation2d.
	 * @param distance The distance to drive forward in meters.
	 */
	public RotateToAndDriveCommand(SwerveDrivetrain drivetrain, Rotation2d angle, Translation2d distance) {
		addCommands(
				new RotateToCommand(drivetrain, angle),
				new DriveDistanceCommand(drivetrain, distance, false));
	}

}
