// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.DriveDurationCommand;
import frc.robot.commands.RotateByCommand;
import frc.robot.commands.SingularSwerveModuleCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ConstantDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	// private final SwerveModule module1 = new SwerveModule(
	// 		SwerveDriveConstants.ANGULAR_MOTOR_ID,
	// 		SwerveDriveConstants.VELOCITY_MOTOR_ID,
	// 		SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID);

	private final AHRS gyro = new AHRS(I2C.Port.kMXP);
	// link for gyro https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(
			gyro);
	private final ConstantDriveCommand test = new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0.01, 0));
	// private final Command setModule = new SingularSwerveModuleCommand(module1, Math.PI / 2, 1);
	// private final Command zeroModule = new SingularSwerveModuleCommand(module1, 0, 0);
	private final Command stopCommand = new StopCommand(drivetrain);
	private final Command driveDistanceTest = new DriveDistanceCommand(drivetrain, new Translation2d(1, 1), true);
	private final Command driveDurationTest = new DriveDurationCommand(drivetrain, 3,
			new ChassisSpeeds(1, 0, Math.PI * 2));
	private final Command rotateTest = new RotateByCommand(drivetrain, new Rotation2d(Math.PI / 2));

	private final CommandJoystick driverJoystick = new CommandJoystick(OperatorConstants.DRIVER_JOYSTICK_PORT);

	private final Command toggleFieldRelative = new RunCommand(drivetrain::toggleFieldRelative, drivetrain);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverJoystick));
		// Configure the trigger bindings
		configureBindings();

		SmartDashboard.putData(CommandScheduler.getInstance());
		// SmartDashboard.putData(module1);

	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		// new Trigger(exampleSubsystem::exampleCondition)
		// 		.onTrue(new ExampleCommand(exampleSubsystem));
		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
		// cancelling on release.
		// driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

		driverJoystick.button(3).onTrue(toggleFieldRelative);
		driverJoystick.button(2).onTrue(stopCommand);

		// Test bindings
		// driverJoystick.button(1).onTrue(setModule);
		// driverJoystick.button(2).onTrue(zeroModule);

		driverJoystick.button(5).onTrue(driveDistanceTest);
		driverJoystick.button(6).onTrue(driveDurationTest);
		driverJoystick.button(4).onTrue(rotateTest);

		driverJoystick.button(7).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0.01, 0)));
		driverJoystick.button(8).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, -0.01, 0)));
		driverJoystick.button(9).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0.01, 0, 0)));
		driverJoystick.button(10).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(-0.01, 0, 0)));
		driverJoystick.button(11).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0, 0.01)));
		driverJoystick.button(12).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0, -0.01)));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return Autos.exampleAuto(exampleSubsystem);
		return null;
	}
}
