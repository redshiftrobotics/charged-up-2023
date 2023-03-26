package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveDrivetrain;

public class GoToTagCommand extends RotateAndDriveSimultaneousCommand {
	SwerveDrivetrain drivetrain;

	public static GoToTagCommand createGoToTagCommand(SwerveDrivetrain drivetrain, Transform3d aprilTag,
			Translation2d desiredDistToTag) {

		Rotation2d tagRotation = new Rotation2d(-aprilTag.getRotation().getY());

		Translation2d driveDistance = new Translation2d(aprilTag.getX(), aprilTag.getZ());

		driveDistance = driveDistance.plus(desiredDistToTag.rotateBy(tagRotation));

		return new GoToTagCommand(drivetrain, tagRotation, driveDistance);
	}

	public GoToTagCommand(SwerveDrivetrain drivetrain, Rotation2d tagRotation, Translation2d driveDistance) {
		super(drivetrain, tagRotation, false, driveDistance, false);

	}

}
