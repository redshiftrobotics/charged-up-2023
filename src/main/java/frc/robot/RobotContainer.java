// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;

import frc.robot.commands.SetArmDegreeCommand;
import frc.robot.commands.SetTurboMode;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ToggleFieldRelativeCommand;
import frc.robot.subsystems.TopArm;
import frc.robot.subsystems.BottomArm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveCamera;
import frc.robot.subsystems.ArmManager;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.DriveDurationCommand;
import frc.robot.commands.FullTestAuto;
import frc.robot.commands.RotateByCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ChangeArmDegreeByCommand;
import frc.robot.commands.ConstantDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.ClawEjectCommand;
import frc.robot.commands.ClawGrabCommand;
import frc.robot.commands.ClawStopCommand;

// import frc.robot.commands.ToggleIntakeCommand;
// import frc.robot.subsystems.Intake;

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
	// private final Command setModule = new SingularSwerveModuleCommand(module1, Math.PI / 2, 1);
	// private final Command zeroModule = new SingularSwerveModuleCommand(module1, 0, 0);

	private final SwerveModule swerveModuleFL = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_FL,
			SwerveDriveConstants.VELOCITY_MOTOR_ID_FL,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL);
	private final SwerveModule swerveModuleFR = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_FR,
			SwerveDriveConstants.VELOCITY_MOTOR_ID_FR,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR);
	private final SwerveModule swerveModuleBL = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_BL,
			SwerveDriveConstants.VELOCITY_MOTOR_ID_BL,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL);
	private final SwerveModule swerveModuleBR = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_BR,
			SwerveDriveConstants.VELOCITY_MOTOR_ID_BR,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR);

	private final AHRS gyro = new AHRS(I2C.Port.kMXP);

	private final DriveCamera driveCamera = new DriveCamera();

	// link for gyro https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL,
			swerveModuleFR, swerveModuleBL, swerveModuleBR);
	private final ConstantDriveCommand test = new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0.01, 0));
	private final Command stopCommand = new StopCommand(drivetrain);
	private final Command driveDistanceTest = new DriveDistanceCommand(drivetrain, new Translation2d(1, 1), true);
	private final Command driveDurationTest = new DriveDurationCommand(drivetrain, 3,
			new ChassisSpeeds(1, 0, Math.PI * 2));
	private final Command rotateTest = new RotateByCommand(drivetrain, new Rotation2d(Math.PI / 2));

	private final Command testAuto = new DriveDurationCommand(drivetrain, 2, 0.05);

	private final CommandJoystick driverJoystick = new CommandJoystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
	// private final Intake intake = new Intake(IntakeConstants.TOP_MOTOR_ID, IntakeConstants.BOTTOM_MOTOR_ID);
	private final CommandJoystick operatorJoystick = new CommandJoystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

	private final Command toggleFieldRelative = new ToggleFieldRelativeCommand(drivetrain);

	// Initialize the bottom arm and top arm
	private final BottomArm bottomArm = new BottomArm(
			ArmConstants.BOTTOM_ARM_MOTOR_ONE_ID,
			ArmConstants.BOTTOM_ARM_MOTOR_TWO_ID,
			ArmConstants.BOTTOM_ARM_ENCODER_ID,
			ArmConstants.BOTTOM_ARM_MIN_DEGREE,
			ArmConstants.BOTTOM_ARM_MAX_DEGREE);
	private final TopArm topArm = new TopArm(
			ArmConstants.TOP_ARM_MOTOR_ID,
			ArmConstants.TOP_ARM_ENCODER_ID,
			ArmConstants.TOP_ARM_MIN_DEGREE,
			ArmConstants.TOP_ARM_MAX_DEGREE);

	// Initialize the arm manager so that setting the degrees of both arms is simpler
	private final ArmManager armManager = new ArmManager(bottomArm, topArm);

	// Commands to set the state of the arm manager 
	// private final Command armDriveCommand = new SetArmDegreeCommand(armManager, 10, 10);
	// private final Command armInspectionComand = new SetArmDegreeCommand(armManager, 60, 10);
	// private final Command armIntakeLowCommand = new SetArmDegreeCommand(armManager, 75, 280);
	// private final Command armIntakeHighCommand = new SetArmDegreeCommand(armManager, 75, 120);
	// private final Command armScoreThreeCommand = new SetArmDegreeCommand(armManager, 45, 180);
	// private final Command armScoreTwoCommand = new SetArmDegreeCommand(armManager, 90, 90);

	private final Command armStowCommand = new SetArmDegreeCommand(armManager, 324, 0);
	private final Command armIntake = new SetArmDegreeCommand(armManager, 296, 0);
	private final Command armTopRung = new SetArmDegreeCommand(armManager, 271, 0);

	private final Command armTestCommand = new SetArmDegreeCommand(armManager, 90, 90);

	private final Command turboModeOn = new SetTurboMode(drivetrain, true);
	private final Command turboModeOff = new SetTurboMode(drivetrain, false);

	private final Command armByCommandTestForward = new ChangeArmDegreeByCommand(armManager, 0, 10);
	private final Command armByCommandTestBackward = new ChangeArmDegreeByCommand(armManager, 0, -10);

	private final Claw claw = new Claw();
	private final Command ClawEjectCommand = new ClawEjectCommand(claw);
	private final Command ClawGrabCommand = new ClawGrabCommand(claw);
	private final Command ClawStopCommand = new ClawStopCommand(claw);
	// private final Command ClawConeKeepCommand = new CLaw

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverJoystick));
		// Configure the trigger bindings
		configureBindings();

		SmartDashboard.putData(CommandScheduler.getInstance());
		// SmartDashboard.putData(module1);
		gyro.zeroYaw();
		SmartDashboard.putNumber("Balance P", 0);
		SmartDashboard.putNumber("Balance I", 0);
		SmartDashboard.putNumber("Balance D", 0);
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
		// driverJoystick.button(3).onTrue(toggleFieldRelative);

		// driverJoystick.button(4).onTrue(armDriveCommand);
		// driverJoystick.button(5).onTrue(armInspectionComand);
		// driverJoystick.button(6).onTrue(armIntakeLowCommand);
		// driverJoystick.button(7).onTrue(armIntakeHighCommand);
		// driverJoystick.button(8).onTrue(armScoreThreeCommand);
		// driverJoystick.button(9).onTrue(armScoreTwoCommand);

		// driverJoystick.button(10).onTrue(armTestCommand);

		// driverJoystick.button(OperatorConstants.TOGGLE_INTAKE_BUTTON_ID).onTrue(new ToggleIntakeCommand(intake));

		// driverJoystick.button(3).onTrue(toggleFieldRelative);
		// driverJoystick.button(2).whileTrue(stopCommand);

		// Test bindings
		// driverJoystick.button(1).onTrue(setModule);
		// driverJoystick.button(2).onTrue(zeroModule);

		// driverJoystick.button(5).onTrue(driveDistanceTest);
		// driverJoystick.button(6).onTrue(driveDurationTest);
		// driverJoystick.button(4).onTrue(rotateTest);

		// driverJoystick.button(10).onTrue(armByCommandTestForward);
		// driverJoystick.button(11).onTrue(armByCommandTestBackward);

		// driverJoystick.button(7).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0.01, 0)));
		// driverJoystick.button(8).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, -0.01, 0)));
		// driverJoystick.button(9).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(-0.01, 0, 0)));
		// driverJoystick.button(10).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0.01, 0, 0)));
		// driverJoystick.button(11).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0, 0.01)));
		// driverJoystick.button(12).whileTrue(new ConstantDriveCommand(drivetrain, new ChassisSpeeds(0, 0, -0.01)));

		driverJoystick.button(1).onTrue(turboModeOn);
		driverJoystick.button(1).onFalse(turboModeOff);

		operatorJoystick.button(7).onTrue(ClawGrabCommand);
		operatorJoystick.button(9).onTrue(ClawEjectCommand);
		operatorJoystick.button(8).onTrue(ClawStopCommand);

		operatorJoystick.button(3).onTrue(armStowCommand);
		operatorJoystick.button(4).onTrue(armIntake);
		operatorJoystick.button(6).onTrue(armTopRung);

		operatorJoystick.button(5).onTrue(new BalanceCommand(drivetrain));

		// operatorJoystick.button(3).onTrue(new BalanceCommand(drivetrain));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return Autos.exampleAuto(exampleSubsystem);
		return new FullTestAuto(drivetrain);
	}
}
