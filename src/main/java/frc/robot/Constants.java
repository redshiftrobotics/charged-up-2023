// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.opencv.core.Scalar;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final double periodicFrequency = 0.02;

	public static class OperatorConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;
		public static final int TOGGLE_INTAKE_BUTTON_ID = 2;
	}

	public static final class CameraConstants {

		// Understading the focal length values: https://en.wikipedia.org/wiki/Camera_resectioning, https://en.wikipedia.org/wiki/Cardinal_point_(optics)

		// Microsoft LifeCam HD-3000 stuff https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
		public static final int CAMERA_RESOLUTION_WIDTH = 640;
		public static final int CAMERA_RESOLUTION_HEIGHT = 480;

		// public static final int CAMERA_RESOLUTION_WIDTH = 1280;
		// public static final int CAMERA_RESOLUTION_HEIGHT = 720;

		public static final double CAMERA_FOCAL_LENGTH_X = 678.154f;
		public static final double CAMERA_FOCAL_LENGTH_Y = 678.17;

		public static final double CAMERA_FOCAL_CENTER_X = 318.135;
		public static final double CAMERA_FOCAL_CENTER_Y = 228.374;

		// LimeLight stuff https://docs.limelightvision.io/en/latest/vision_pipeline_tuning.html
		// public static final int CAMERA_RESOLUTION_WIDTH = 960;
		// public static final int CAMERA_RESOLUTION_HEIGHT = 720;

		// public static final double CAMERA_FOCAL_LENGTH_X = 772.53876202;
		// public static final double CAMERA_FOCAL_LENGTH_Y = 769.052151477;

		// public static final double CAMERA_FOCAL_CENTER_X = 479.132337442;
		// public static final double CAMERA_FOCAL_CENTER_Y = 359.143001808;

		// width and height of April Tag
		public static final double APRIL_TAG_SIZE_MM = 152.4;

		// What Gaussian blur should be applied to the segmented image
		// important for noisy images
		public static final float QUAD_SIGMA = 0.8f;

		// if a cluster of pixels is smaller than this value it is ignored
		public static final int MIN_CLUSTER_PIXELS = 250;

		// The yaw rotation of an object before it is ignored
		public static final double CRITICAL_ANGLE = Math.toRadians(45);

		// How square an object needs to be to be considered
		public static final float MAX_LINE_FIT_MSE = 15f;

		// the April Tag family we are using
		public static final String TAG_FAMILY = "tag16h5";

		public static final Transform3d CAMERA_POSITION = new Transform3d(new Translation3d(0, 0.5, 0.5),
				new Rotation3d());
	}

	public static final class VideoDisplayConstants {
		public static final Scalar RED = new Scalar(0, 0, 255);
		public static final Scalar GREEN = new Scalar(0, 255, 0);
		public static final Scalar BLUE = new Scalar(255, 0, 0);
		public static final Scalar WHITE = new Scalar(0, 0, 0);
		public static final Scalar BLACK = new Scalar(255, 255, 255);

		public static final Scalar TEXT_COLOR = VideoDisplayConstants.GREEN;
		public static final Scalar BOX_OUTLINE_COLOR = VideoDisplayConstants.RED;
	}

	public static class RobotConstants {
		// in inches
		public static final double BUMPER_WIDTH = 0;
		public static final double ROBOT_WIDTH = 24 + BUMPER_WIDTH * 2;
		public static final double ROBOT_HEIGHT = 36 + BUMPER_WIDTH * 2;
	}

	public static class FieldConstants {
		// IN = inches, no unit in name means it's in meters
		// All translation2ds should be in meters
		// Inches to meters constant
		private static final double IN_TO_M = 0.0254;
		public static final double FIELD_LENGTH_IN = 651.25;
		public static final double FIELD_WIDTH_IN = 315.5;
		public static final double FIELD_LENGTH = FIELD_LENGTH_IN * IN_TO_M;
		public static final double FIELD_WIDTH = FIELD_WIDTH_IN * IN_TO_M;

		public static final Map<Integer, Translation2d> TAGS = Map.of(
				1, new Translation2d(610.77 * IN_TO_M, 42.19 * IN_TO_M),
				2, new Translation2d(610.77 * IN_TO_M, 108.19 * IN_TO_M),
				3, new Translation2d(610.77 * IN_TO_M, 174.19 * IN_TO_M),
				4, new Translation2d(636.96 * IN_TO_M, 265.74 * IN_TO_M),
				5, new Translation2d(14.25 * IN_TO_M, 265.74 * IN_TO_M),
				6, new Translation2d(40.45 * IN_TO_M, 174.19 * IN_TO_M),
				7, new Translation2d(40.45 * IN_TO_M, 108.19 * IN_TO_M),
				8, new Translation2d(40.45 * IN_TO_M, 42.19 * IN_TO_M));

		public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
		// Staging marks are from bottom to top (1 is bottom most 4 is top most)
		public static final Translation2d RED_STAGING_MARK_1 = new Translation2d(386.77 * IN_TO_M, 36.19 * IN_TO_M);
		public static final Translation2d RED_STAGING_MARK_2 = new Translation2d(386.77 * IN_TO_M, 84.19 * IN_TO_M);
		public static final Translation2d RED_STAGING_MARK_3 = new Translation2d(386.77 * IN_TO_M, 132.19 * IN_TO_M);
		public static final Translation2d RED_STAGING_MARK_4 = new Translation2d(386.77 * IN_TO_M, 180.19 * IN_TO_M);

		public static final Translation2d BLUE_STAGING_MARK_1 = new Translation2d(264.45 * IN_TO_M, 36.19 * IN_TO_M);
		public static final Translation2d BLUE_STAGING_MARK_2 = new Translation2d(264.45 * IN_TO_M, 84.19 * IN_TO_M);
		public static final Translation2d BLUE_STAGING_MARK_3 = new Translation2d(264.45 * IN_TO_M, 132.19 * IN_TO_M);
		public static final Translation2d BLUE_STAGING_MARK_4 = new Translation2d(264.45 * IN_TO_M, 180.19 * IN_TO_M);

		public static final Translation2d BLUE_CHARGE_STATION = new Translation2d(137.2 * IN_TO_M, 107.39 * IN_TO_M);
		public static final Translation2d RED_CHARGE_STATION = new Translation2d(514.05 * IN_TO_M, 107.39 * IN_TO_M);

		// nodes are from bottom to top, currently only marking the middle nodes for each group
		public static final Translation2d RED_GRID_NODE_2 = new Translation2d(593.77 * IN_TO_M, 42.19 * IN_TO_M);
		public static final Translation2d RED_GRID_NODE_5 = new Translation2d(593.77 * IN_TO_M, 108.19 * IN_TO_M);
		public static final Translation2d RED_GRID_NODE_8 = new Translation2d(593.77 * IN_TO_M, 174.19 * IN_TO_M);

		public static final Translation2d BLUE_GRID_NODE_2 = new Translation2d(57.45 * IN_TO_M, 42.19 * IN_TO_M);
		public static final Translation2d BLUE_GRID_NODE_5 = new Translation2d(57.45 * IN_TO_M, 108.19 * IN_TO_M);
		public static final Translation2d BLUE_GRID_NODE_8 = new Translation2d(57.45 * IN_TO_M, 174.19 * IN_TO_M);

		public static final Translation2d RED_SUBSTATION = new Translation2d(636.96 * IN_TO_M, 265.74 * IN_TO_M);
		public static final Translation2d BLUE_SUBSTATION = new Translation2d(14.25 * IN_TO_M, 265.74 * IN_TO_M);
	}

	public static class SwerveDriveConstants {

		public static double JOYSTICK_DEADZONE = 0.05;
		// Meters per second
		public static final double MAX_SPEED = 0.01;

		// Radians per second
		public static final double MAX_ROTATION_SPEED = Math.PI / 300;

		public static final double MODULE_LOCATION_X = 0.25;
		public static final double MODULE_LOCATION_Y = 0.25;

		public static final Translation2d DESIRED_DIST_TO_APRILTAG = new Translation2d(-0.75, 0);

		// TESTING ONLY
		public static final int ANGULAR_MOTOR_ID = 18;
		public static final int VELOCITY_MOTOR_ID = 19;
		public static final int ANGULAR_MOTOR_ENCODER_ID = 28;

		// TODO update motor IDs
		public static final int ANGULAR_MOTOR_ID_FL = 7;
		public static final int VELOCITY_MOTOR_ID_FL = 8;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL = 26;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_FL = 155 - 2;

		public static final int ANGULAR_MOTOR_ID_FR = 5;
		public static final int VELOCITY_MOTOR_ID_FR = 6;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR = 27;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_FR = 316 - 180 - 3;

		public static final int ANGULAR_MOTOR_ID_BL = 2;
		public static final int VELOCITY_MOTOR_ID_BL = 1;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL = 25;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_BL = 225 + 36;

		public static final int ANGULAR_MOTOR_ID_BR = 18;
		public static final int VELOCITY_MOTOR_ID_BR = 19;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR = 28;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_BR = 270 + 16;

		public static final double ENCODER_NATIVE_NUM_SUBDIVISION = 42;
		public static final double ROTATIONAL_UNITS_CONSTANT = 2 * Math.PI;

		public static final double ENCODER_POSITION_CONVERSION_FACTOR = ROTATIONAL_UNITS_CONSTANT
				/ ENCODER_NATIVE_NUM_SUBDIVISION;

		public static final double WHEEL_DIAMETER_INCHES = 7;
		public static final double WHEEL_DIAMETER_METRIC = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METRIC * Math.PI;

		// https://www.swervedrivespecialties.com/products/mk4-swerve-module 
		// It depends on which version we bought
		// See above site for info
		public static final double VELOCITY_MOTOR_GEAR_RATIO = 57 / 7;

		public static final double ANGULAR_MOTOR_GEAR_RATIO = 12.8;
		public static final double ANGULAR_ENCODER_GEAR_RATIO = 1;

		public static final double ANGULAR_PID_P = 0.005;
		public static final double ANGULAR_PID_I = 0.0;
		public static final double ANGULAR_PID_D = 0;

		public static final double VELOCITY_PID_P = 0.000001;
		public static final double VELOCITY_PID_I = 0;
		public static final double VELOCITY_PID_D = 0;
		public static final double VELOCITY_PID_FF = 0.090944883322;

		public static final double ROBOT_VELOCITY_PID_P = 0.05;
		public static final double ROBOT_VELOCITY_PID_I = 0;
		public static final double ROBOT_VELOCITY_PID_D = 0;

		public static final double ROBOT_ANGULAR_PID_P = 0.05;
		public static final double ROBOT_ANGULAR_PID_I = 0;
		public static final double ROBOT_ANGULAR_PID_D = 0;

		// The maximum speed and error the robot will stop at for DriveDistanceCommand.
		public static final double ROBOT_DISTANCE_TOLERANCE = 0.1;
		public static final double ROBOT_STOP_VELOCITY_TOLERANCE = 0.1;

		// The maximum rotation speed and error the robot will stop at for RotateByCommand.
		public static final double ROBOT_ANGLE_TOLERANCE = 0.1;
		public static final double ROBOT_STOP_ROTATION_TOLERANCE = 0.1;
	}

	public static class IntakeConstants {
		public static final double INTAKE_MOTOR_SPEED = 0.5;
		public static final int TOP_MOTOR_ID = 0;
		public static final int BOTTOM_MOTOR_ID = 1;
	}

}
