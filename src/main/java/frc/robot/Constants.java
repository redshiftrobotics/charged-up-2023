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

	public static final Pose2d STARTING_POSITION = new Pose2d(0, 0, new Rotation2d(0)); // x, y, theta

	public static final Pose2d CHARGE_STATION_POSITION = new Pose2d(0, 0, new Rotation2d(0));

	public static final double CENTER_OF_MASS_OFFSET = 0;

	public static class VisionConstants {
		public static final int CAMERA_PORT = 0;

		// Most camera related stuff happens on Pi co-processor

		// TODO set values
		public static final Transform3d CAMERA_POSITION_FROM_CENTER_CENTER_MM = new Transform3d(
				new Translation3d(-317.5, 0, -457.2),
				new Rotation3d());

		public static final int ROBOT_WIDTH_MM = 100;
		public static final int ROBOT_LENGTH_MM = 150;

		public static final int ROBOT_HALF_WIDTH_MM = ROBOT_WIDTH_MM / 2;
		public static final int ROBOT_HALF_LENGTH_MM = ROBOT_LENGTH_MM / 2;
	}

	public static class OperatorConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;
		public static final int TOGGLE_INTAKE_BUTTON_ID = 2;
	}

	public static class SwerveDriveConstants {

		public static double JOYSTICK_DEADZONE = 0.2;

		// Clamps Final Speed, in Percent
		public static final double MAX_SPEED = 0.75;

		// Percent of Speed use by Joystick
		public static final double MAX_NORMAL_JOYSTICK_SPEED = 0.08;
		public static final double MAX_TURBO_JOYSTICK_SPEED = 0.2;
		public static final double MAX_FIRST_ROTATE_JOYSTICK_SPEED = 0.4;
		public static final double MAX_SECOND_ROTATE_JOYSTICK_SPEED = 0.2;

		// Radians per second
		// public static final double MAX_ROTATION_SPEED = 0.02;

		public static final double MODULE_LOCATION_X = 0.25;
		public static final double MODULE_LOCATION_Y = 0.25;

		public static final Translation2d DESIRED_DIST_TO_APRILTAG = new Translation2d(-0.75, 0);

		// TESTING ONLY
		// public static final int ANGULAR_MOTOR_ID = 18;
		// public static final int VELOCITY_MOTOR_ID = 19;
		// public static final int ANGULAR_MOTOR_ENCODER_ID = 28;

		// TODO update motor IDs
		public static final int ANGULAR_MOTOR_ID_FL = 13;
		public static final int VELOCITY_MOTOR_ID_FL = 12;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL = 26;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_FL = -42 + 90 - 90;

		public static final int ANGULAR_MOTOR_ID_FR = 8;
		public static final int VELOCITY_MOTOR_ID_FR = 15;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR = 27;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_FR = -22 + 90 - 90;

		public static final int ANGULAR_MOTOR_ID_BL = 2;
		public static final int VELOCITY_MOTOR_ID_BL = 31;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL = 25;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_BL = -257 - 90 - 90;

		public static final int ANGULAR_MOTOR_ID_BR = 1;
		public static final int VELOCITY_MOTOR_ID_BR = 9;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR = 28;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_BR = -345 + 90 - 90;

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

		public static final double ANGULAR_PID_P = 0.0075;
		public static final double ANGULAR_PID_I = 0.0;
		public static final double ANGULAR_PID_D = 0;

		public static final double VELOCITY_PID_P = 0.000001;
		public static final double VELOCITY_PID_I = 0;
		public static final double VELOCITY_PID_D = 0;
		public static final double VELOCITY_PID_FF = 0.090944883322;

		public static final double ROBOT_VELOCITY_PID_P = 0.05;
		public static final double ROBOT_VELOCITY_PID_I = 0;
		public static final double ROBOT_VELOCITY_PID_D = 0;

		public static final double ROBOT_ANGULAR_PID_P = -0.2;
		public static final double ROBOT_ANGULAR_PID_I = 0;
		public static final double ROBOT_ANGULAR_PID_D = -0.005;

		// The maximum speed and error the robot will stop at for DriveDistanceCommand.
		public static final double ROBOT_DISTANCE_TOLERANCE = 0.5;
		public static final double ROBOT_STOP_VELOCITY_TOLERANCE = .05;

		// The maximum rotation speed and error the robot will stop at for RotateByCommand.
		public static final double ROBOT_ANGLE_TOLERANCE = 0.1;
		public static final double ROBOT_STOP_ROTATION_TOLERANCE = 0.1;
	}

	public static class IntakeConstants {
		public static final double INTAKE_MOTOR_SPEED = 0.5;
		public static final int TOP_MOTOR_ID = 0;
		public static final int BOTTOM_MOTOR_ID = 1;
	}

	public static class ArmConstants {
		public static final double TOP_ARM_START_DEGREE = 332;
		public static final double BOTTOM_ARM_START_DEGREE = 333;

		public static final double BOTTOM_ARM_PID_P = -4;
		public static final double BOTTOM_ARM_PID_I = 0;
		public static final double BOTTOM_ARM_PID_D = -1;

		public static final double TOP_ARM_PID_P = -4;
		public static final double TOP_ARM_PID_I = 0;
		public static final double TOP_ARM_PID_D = 0;

		public static final double ARM_MOTOR_GEAR_RATIO = 1;

		public static final int BOTTOM_ARM_MOTOR_ONE_ID = 3;
		public static final int BOTTOM_ARM_MOTOR_TWO_ID = 10;
		public static final int TOP_ARM_MOTOR_ID = 7;
		public static final int BOTTOM_ARM_ENCODER_ID = 29;
		public static final int TOP_ARM_ENCODER_ID = 30;

		public static final double BOTTOM_ARM_MIN_DEGREE = 313;
		public static final double TOP_ARM_MIN_DEGREE = 350;
		public static final double BOTTOM_ARM_MAX_DEGREE = 116;
		public static final double TOP_ARM_MAX_DEGREE = 211;

		public static final double BOTTOM_ARM_PID_S = 0.0001;
		public static final double BOTTOM_ARM_PID_G = 2.04;
		public static final double BOTTOM_ARM_PID_V = 3.12;
		public static final double BOTTOM_ARM_PID_A = .11;

		public static final double TOP_ARM_PID_S = 0.0001;
		public static final double TOP_ARM_PID_G = 2.14;
		public static final double TOP_ARM_PID_V = 3.12;
		public static final double TOP_ARM_PID_A = .22;

		public static final double BOTTOM_ARM_FEEDFORWARD_POS = 0;
		public static final double BOTTOM_ARM_FEEDFORWARD_VEL = 1;
		public static final double BOTTOM_ARM_FEEDFORWARD_ACCEL = 0;

		public static final double TOP_ARM_FEEDFORWARD_POS = 0;
		public static final double TOP_ARM_FEEDFORWARD_VEL = 1;
		public static final double TOP_ARM_FEEDFORWARD_ACCEL = 0;

		public static final double TOP_ARM_MAX_VELOCITY = 0.1;
		public static final double TOP_ARM_MAX_ACCEL = .01;

		public static final double BOTTOM_ARM_MAX_VELOCITY = 0.1;
		public static final double BOTTOM_ARM_MAX_ACCEL = .01;
	}

}
