// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that runs swerve drive based on joystick input for the robot. */
public class SwerveDriveCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final CommandJoystick joystick;

	/**
	 * Creates a new SwerveDriveCommand.
	 *
	 * @param drivetrain The drivetrain used by this command.
	 * @param joystick The joystick used by this command.
	 */
	public SwerveDriveCommand(SwerveDrivetrain drivetrain, CommandJoystick joystick) {
		this.drivetrain = drivetrain;
		this.joystick = joystick;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	// @Override
	// public void initialize() {
	// }

	private double deadbandJoystick(double speed) {
		if (speed < SwerveDriveConstants.JOYSTICK_DEADZONE && speed > -SwerveDriveConstants.JOYSTICK_DEADZONE) {
			speed = 0;
		} else {
			double scale = 1 / (1 - SwerveDriveConstants.JOYSTICK_DEADZONE);
			if (speed > 0) {
				speed -= SwerveDriveConstants.JOYSTICK_DEADZONE;
				speed *= scale;
			} else {
				speed += SwerveDriveConstants.JOYSTICK_DEADZONE;
				speed *= scale;
			}
		}
		return speed;
	}

	// Called every time the scheduler runs while the command is scheduled.
	// Gets joystick input and sends it to the drivetrain
	@Override
	public void execute() {

		double speedX = deadbandJoystick(-joystick.getX());
		double speedY = deadbandJoystick(-joystick.getY());
		double speedR = 0;

		if (joystick.button(7).getAsBoolean()) {
			speedR = 0.25 * SwerveDriveConstants.MAX_FIRST_ROTATE_JOYSTICK_SPEED;
		} else if (joystick.button(8).getAsBoolean()) {
			speedR = -0.25 * SwerveDriveConstants.MAX_FIRST_ROTATE_JOYSTICK_SPEED;
		} else if (joystick.button(9).getAsBoolean()) {
			speedR = 0.25 * SwerveDriveConstants.MAX_SECOND_ROTATE_JOYSTICK_SPEED;
		} else if (joystick.button(10).getAsBoolean()) {
			speedR = -0.25 * SwerveDriveConstants.MAX_SECOND_ROTATE_JOYSTICK_SPEED;
		}

		ChassisSpeeds speeds;
		if (drivetrain.getTurboMode()) {
			speeds = new ChassisSpeeds(
					speedY * SwerveDriveConstants.MAX_TURBO_JOYSTICK_SPEED,
					speedX * SwerveDriveConstants.MAX_TURBO_JOYSTICK_SPEED,
					speedR);
		} else {
			speeds = new ChassisSpeeds(
					speedY * SwerveDriveConstants.MAX_NORMAL_JOYSTICK_SPEED,
					speedX * SwerveDriveConstants.MAX_NORMAL_JOYSTICK_SPEED,
					speedR);
		}

		SmartDashboard.putNumber("Joy X", joystick.getX());
		SmartDashboard.putNumber("Joy Y", joystick.getY());
		SmartDashboard.putNumber("Joy R", joystick.getTwist());

		drivetrain.setSwerveModuleStates(speeds);

		SmartDashboard.putBoolean("Turbo Mode", drivetrain.getTurboMode());
	}

	// Called once the command ends or is interrupted.
	// @Override
	// public void end(boolean interrupted) {
	// }

	// Default command so should never finish
	@Override
	public boolean isFinished() {
		return false;
	}
}
