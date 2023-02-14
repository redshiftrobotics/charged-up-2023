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

	public static final Pose2d STARTING_POSITION = new Pose2d(0, 0, new Rotation2d(0)); // x, y, theta

	public static class OperatorConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;
		public static final int TOGGLE_INTAKE_BUTTON_ID = 2;
	}

	public static class SwerveDriveConstants {
		// Meters per second
		public static final double MAX_SPEED = 1;

		// Radians per second
		public static final double MAX_ROTATION_SPEED = Math.PI;

		public static final double MODULE_LOCATION_X = 0.25;
		public static final double MODULE_LOCATION_Y = 0.25;

		public static final Translation2d DESIRED_DIST_TO_APRILTAG = new Translation2d(-0.75, 0);

		// TESTING ONLY
		public static final int ANGULAR_MOTOR_ID = 19;
		public static final int VELOCITY_MOTOR_ID = 17;
		public static final int ANGULAR_MOTOR_ENCODER_ID = 25;

		// TODO update motor IDs
		public static final int ANGULAR_MOTOR_ID_FL = 0;
		public static final int VELOCITY_MOTOR_ID_FL = 1;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL = 8;

		public static final int ANGULAR_MOTOR_ID_FR = 2;
		public static final int VELOCITY_MOTOR_ID_FR = 3;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR = 9;

		public static final int ANGULAR_MOTOR_ID_BL = 4;
		public static final int VELOCITY_MOTOR_ID_BL = 5;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL = 10;

		public static final int ANGULAR_MOTOR_ID_BR = 6;
		public static final int VELOCITY_MOTOR_ID_BR = 7;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR = 11;

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

		public static final double ANGULAR_PID_P = 0.05;
		public static final double ANGULAR_PID_I = 0;
		public static final double ANGULAR_PID_D = 0;

		public static final double VELOCITY_PID_P = 0.05;
		public static final double VELOCITY_PID_I = 0;
		public static final double VELOCITY_PID_D = 0;

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

	public static final class CameraConstants {
		// TODO get camera position from design.
		public static final Transform3d CAMERA_POSITION = new Transform3d(new Translation3d(0, 0.5, 0.5),
				new Rotation3d());

	}

	public static class IntakeConstants {
		public static final double INTAKE_MOTOR_SPEED = 0.5;
		public static final int TOP_MOTOR_ID = 0;
		public static final int BOTTOM_MOTOR_ID = 1;
	}

}
