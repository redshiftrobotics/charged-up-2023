// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SingularSwerveModuleCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	private final SwerveModule module1 = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID,
			SwerveDriveConstants.VELOCITY_MOTOR_ID,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID);

	private final Command setModule = new SingularSwerveModuleCommand(module1, Math.PI / 2, 1);
	private final Command zeroModule = new SingularSwerveModuleCommand(module1, 0, 0);
	// Replace with CommandPS4Controller or CommandJoystick if needed
	// private final CommandXboxController driverController = new CommandXboxController(
	// 		OperatorConstants.DRIVER_CONTROLLER_PORT);

	private final CommandJoystick driverJoystick = new CommandJoystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
	// private final Intake intake = new Intake(IntakeConstants.TOP_MOTOR_ID, IntakeConstants.BOTTOM_MOTOR_ID);

	//  Initialize drivetrain object - AHRS is the class for the gyroscope
	// private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(new AHRS());

	// private final Command toggleFieldRelative = new RunCommand(drivetrain::toggleFieldRelative, drivetrain);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverJoystick));
		// Configure the trigger bindings
		configureBindings();

		SmartDashboard.putData(CommandScheduler.getInstance());
		SmartDashboard.putData(module1);

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
		driverJoystick.button(1).onTrue(setModule);
		driverJoystick.button(2).onTrue(zeroModule);
		// driverJoystick.button(3).onTrue(toggleFieldRelative);

		// driverJoystick.button(OperatorConstants.TOGGLE_INTAKE_BUTTON_ID).onTrue(new ToggleIntakeCommand(intake));

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
