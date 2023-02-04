// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
	public static class OperatorConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;
	}

	public static class SwerveDriveConstants {

		public static final int ANGULAR_MOTOR_ENCODER_ID = 0;

		public static final int ANGULAR_MOTOR_ID = 3;
		public static final int VELOCITY_MOTOR_ID = 19;

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

		public static final double ANGULAR_PID_P = 1;
		public static final double ANGULAR_PID_I = 0;
		public static final double ANGULAR_PID_D = 0;

		public static final double VELOCITY_PID_P = 1;
		public static final double VELOCITY_PID_I = 0;
		public static final double VELOCITY_PID_D = 0;
	}

	public static final class CameraConstants {
		public static final int CAMERA_RESOLUTION_WIDTH = 640;
		public static final int CAMERA_RESOLUTION_HEIGHT = 480;

		// Lime Light stuffhttps://docs.limelightvision.io/en/latest/vision_pipeline_tuning.html
		// public static final int CAMERA_RESOLUTION_WIDTH = 960;
		// public static final int CAMERA_RESOLUTION_HEIGHT = 720;

		public static final double CAMERA_FOCAL_CENTER_X = 479.132337442;
		public static final double CAMERA_FOCAL_CENTER_Y = 359.143001808;

		public static final double CAMERA_FOCAL_LENGTH_X = 772.53876202;
		public static final double CAMERA_FOCAL_LENGTH_Y = 769.052151477;

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
	}
}
