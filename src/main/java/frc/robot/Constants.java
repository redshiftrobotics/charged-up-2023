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
}
