package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
		Transform3d tagPose = camera.getDetectedAprilTagById(desiredTag);

		tagPose = tagPose.plus(CameraConstants.CAMERA_POSITION);
		Rotation2d tagRotation = new Rotation2d(-tagPose.getRotation().getY());
		Translation2d driveDistance = new Translation2d(tagPose.getX(), tagPose.getZ());
		driveDistance = driveDistance.plus(desiredDistToTag.rotateBy(tagRotation));

		// final Transform3d tagPose = camera.getDetectedAprilTagById(desiredTag);
		// final Transform2d tagPose2d = camera.makePose2d(tagPose);

		// drivetrain.setSwerveModuleStates(
		// 		new ChassisSpeeds(tagPose2d.getX(), tagPose2d.getY(), tagPose2d.getRotation().getRadians()));

	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
