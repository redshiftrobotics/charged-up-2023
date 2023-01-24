// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO update file to be field-oriented. Is same as SwerveDriveCommand now.

package frc.robot.commands;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that runs swerve drive based on joystick input for the robot. */
public class SwerveDriveFieldOrientedCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final CommandJoystick joystick;

	/**
	 * Creates a new SwerveDriveFieldOrientedCommand.
	 *
	 * @param drivetrain The drivetrain used by this command.
	 * @param joystick The joystick used by this command.
	 */
	public SwerveDriveFieldOrientedCommand(SwerveDrivetrain drivetrain, CommandJoystick joystick) {
		this.drivetrain = drivetrain;
		this.joystick = joystick;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	// @Override
	// public void initialize() {
	// }

	// Called every time the scheduler runs while the command is scheduled.
	// Gets joystick input and sends it to the drivetrain
	@Override
	public void execute() {
		ChassisSpeeds speeds = new ChassisSpeeds(
				-joystick.getX() * SwerveDriveConstants.MAX_SPEED,
				joystick.getY() * SwerveDriveConstants.MAX_SPEED,
				joystick.getTwist() * SwerveDriveConstants.MAX_ROTATION_SPEED);
		drivetrain.setSwerveModuleStates(speeds);
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
