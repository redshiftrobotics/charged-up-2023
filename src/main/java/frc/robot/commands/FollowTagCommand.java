package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrivetrain;

public class FollowTagCommand extends CommandBase {

	public final SwerveDrivetrain drivetrain;
	public final Camera camera;
	public final Translation2d desiredDistToTag;
	public final int desiredTag;

	private final PIDController pidRotation = new PIDController(
			SwerveDriveConstants.ROBOT_ANGULAR_PID_P,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_I,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_D);
	private final PIDController pidVelocity = new PIDController(
			SwerveDriveConstants.ROBOT_VELOCITY_PID_P,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_I,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_D);

	public FollowTagCommand(SwerveDrivetrain drivetrain, Camera camera, Translation2d desiredDistToTag,
			int desiredTag) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		this.desiredDistToTag = desiredDistToTag;
		this.desiredTag = desiredTag;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		Transform3d tagPose = camera.getDetectedAprilTagById(desiredTag);

		tagPose = tagPose.plus(CameraConstants.CAMERA_POSITION);
		Rotation2d tagRotation = new Rotation2d(-tagPose.getRotation().getY());
		Translation2d driveDistance = camera.makePose2d(tagPose);

		driveDistance = driveDistance.plus(desiredDistToTag.rotateBy(tagRotation));

	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {

	}
}
