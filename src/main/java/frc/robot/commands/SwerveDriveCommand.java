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

	// Called every time the scheduler runs while the command is scheduled.
	// Gets joystick input and sends it to the drivetrain
	@Override
	public void execute() {

		double speedX = joystick.getX();
		double speedY = -joystick.getY();
		double speedR = -joystick.getZ();

		if (speedX < SwerveDriveConstants.JOYSTICK_DEADZONE && speedX > -SwerveDriveConstants.JOYSTICK_DEADZONE) {
			speedX = 0;
		}
		if (speedY < SwerveDriveConstants.JOYSTICK_DEADZONE && speedY > -SwerveDriveConstants.JOYSTICK_DEADZONE) {
			speedY = 0;
		}
		if (speedR < SwerveDriveConstants.JOYSTICK_DEADZONE && speedR > -SwerveDriveConstants.JOYSTICK_DEADZONE) {
			speedR = 0;
		}

		ChassisSpeeds speeds = new ChassisSpeeds(
				speedX,
				speedY,
				speedR);
		SmartDashboard.putNumber("Joy X", joystick.getX());
		SmartDashboard.putNumber("Joy Y", joystick.getY());
		SmartDashboard.putNumber("Joy R", joystick.getTwist());

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
