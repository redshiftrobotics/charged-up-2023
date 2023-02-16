package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.networktables.Topic;

public class GoToTagCommand extends RotateAndDriveSimultaneousCommand {
	SwerveDrivetrain drivetrain;

	// TYPESubscriber sub = NetworkTablesInstance.getDefault().getTable("Vision").getTYPETopic("VARIABLE NAME").subscribe(DEFAULT VALUE);
	// sub.get();

	public static GoToTagCommand createGoToTagCommand(SwerveDrivetrain drivetrain, Transform3d aprilTag) {
		// GenericSubscriber sub = NetworkTablesInstance.getDefault().getTable("Vision")
		// 		.getTransform3dTopic("aprilTagPosition").subscribe(new Transform3d());
		// Transform3d aprilTag = sub.get();
		Transform3d tagPose = aprilTag.plus(CameraConstants.CAMERA_POSITION);
		Rotation2d tagRotation = new Rotation2d(-tagPose.getRotation().getY());
		Translation2d driveDistance = new Translation2d(tagPose.getZ(), tagPose.getX());
		driveDistance.plus(SwerveDriveConstants.DESIRED_DIST_TO_APRILTAG.rotateBy(tagRotation));
		return new GoToTagCommand(null, null, null);
	}

	public GoToTagCommand(SwerveDrivetrain drivetrain, Rotation2d tagRotation, Translation2d driveDistance) {
		super(drivetrain, tagRotation, false, driveDistance, false);

	}

}
