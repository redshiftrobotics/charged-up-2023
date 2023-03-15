package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrivetrain;

public class FollowTagCommand extends CommandBase {

	public final SwerveDrivetrain drivetrain;
	public final Camera camera;
	public final Transform2d desiredTransformToTag;
	public final int targetTagId;

	private final PIDController pidRotation = new PIDController(
			SwerveDriveConstants.ROBOT_ANGULAR_PID_P,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_I,
			SwerveDriveConstants.ROBOT_ANGULAR_PID_D);
	private final PIDController pidVelocity = new PIDController(
			SwerveDriveConstants.ROBOT_VELOCITY_PID_P,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_I,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_D);

	public FollowTagCommand(SwerveDrivetrain drivetrain, Camera camera, Transform2d desiredTransformToTag,
			int targetTagId) {
		this.drivetrain = drivetrain;
		this.camera = camera;
		this.desiredTransformToTag = desiredTransformToTag;
		this.targetTagId = targetTagId;

		pidRotation.enableContinuousInput(0, 2 * Math.PI);
		pidRotation.setTolerance(SwerveDriveConstants.ROBOT_ANGLE_TOLERANCE,
				SwerveDriveConstants.ROBOT_STOP_ROTATION_TOLERANCE);
		pidVelocity.setTolerance(SwerveDriveConstants.ROBOT_DISTANCE_TOLERANCE,
				SwerveDriveConstants.ROBOT_STOP_VELOCITY_TOLERANCE);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

		// get 3d pose of tag
		final Transform3d tagTransform3d = camera.getDetectedAprilTagById(targetTagId);

		// Quality link to what is happening:
		// https://docs.google.com/presentation/d/1-SesKPdsZrPE4WUMpFoU9UIIiOYIwO94OG4yPuYZS1s/edit?usp=sharing

		// make fixed tag pose into 2d
		Transform2d tagPose2d = camera.makeTransformInto2d(tagTransform3d);

		// remove the extra distance from being placed a certain distance away from the very front and center of the robot
		tagPose2d = tagPose2d.plus(camera.makeTransformInto2d(CameraConstants.CAMERA_POSITION));

		// create a desired location that accounts for tags rotation
		final Transform2d rotatedDesiredTransformToTag = new Transform2d(
				desiredTransformToTag.getTranslation().rotateBy(tagPose2d.getRotation()),
				desiredTransformToTag.getRotation());

		// get to desired distance away from tag
		tagPose2d = tagPose2d.plus(rotatedDesiredTransformToTag);

		drivetrain.setSwerveModuleStates(
				new ChassisSpeeds(tagPose2d.getX(), tagPose2d.getY(), tagPose2d.getRotation().getRadians()));

	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
