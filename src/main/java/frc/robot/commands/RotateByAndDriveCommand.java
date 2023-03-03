package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 *  Rotate by a given angle and drive forward a given distance in meters.
 */
public class RotateByAndDriveCommand extends SequentialCommandGroup {
	/**
	 * Rotate by a given angle and drive forward a given distance in meters.
	 * @param drivetrain The drivetrain of the robot.
	 * @param angle The angle to rotate by as a Rotation2d.
	 * @param distance The distance to drive forward in meters.
	 */
	public RotateByAndDriveCommand(SwerveDrivetrain drivetrain, Rotation2d angle, Translation2d distance) {
		addCommands(
				new RotateByCommand(drivetrain, angle),
				new DriveDistanceCommand(drivetrain, distance, false));
	}
}
