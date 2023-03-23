package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants.VisionConstants;

public class GoToTagCommand extends RotateAndDriveSimultaneousCommand {
	SwerveDrivetrain drivetrain;

	// TYPESubscriber sub = NetworkTablesInstance.getDefault().getTable("Vision").getTYPETopic("VARIABLE NAME").subscribe(DEFAULT VALUE);
	// sub.get();

	public static GoToTagCommand createGoToTagCommand(SwerveDrivetrain drivetrain, Transform3d aprilTag,
			Transform2d desiredPosFromTag) {
		// GenericSubscriber sub = NetworkTablesInstance.getDefault().getTable("Vision")
		// 		.getTransform3dTopic("aprilTagPosition").subscribe(new Transform3d());
		// Transform3d aprilTag = sub.get();
		Transform3d tagPose = aprilTag.plus(VisionConstants.CAMERA_POSITION_FROM_CENTER_CENTER);
		Rotation2d tagRotation = new Rotation2d(-tagPose.getRotation().getY());
		Translation2d driveDistance = new Translation2d(tagPose.getX(), tagPose.getZ());
		driveDistance = driveDistance.plus(desiredPosFromTag.getTranslation().rotateBy(tagRotation));
		return new GoToTagCommand(drivetrain, tagRotation.plus(desiredPosFromTag.getRotation()), driveDistance);
	}

	public static GoToTagCommand createGoToTagCommand(SwerveDrivetrain drivetrain, Transform3d aprilTag,
			Translation2d desiredDistToTag) {
		// GenericSubscriber sub = NetworkTablesInstance.getDefault().getTable("Vision")
		// 		.getTransform3dTopic("aprilTagPosition").subscribe(new Transform3d());
		// Transform3d aprilTag = sub.get();
		// Transform3d tagPose = aprilTag.plus(CameraConstants.CAMERA_POSITION);
		Transform3d tagPose = aprilTag;
		Rotation2d tagRotation = new Rotation2d(-tagPose.getRotation().getY());
		Translation2d driveDistance = new Translation2d(tagPose.getX(), tagPose.getZ());
		driveDistance = driveDistance.plus(desiredDistToTag.rotateBy(tagRotation));
		return new GoToTagCommand(drivetrain, tagRotation, driveDistance);
	}

	public GoToTagCommand(SwerveDrivetrain drivetrain, Rotation2d tagRelativeRotation, Translation2d driveDistance) {
		super(drivetrain, tagRelativeRotation, false, driveDistance, false);

	}

}
