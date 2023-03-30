// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BalenceConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class BalanceCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;

	// TODO figure what constants to use 
	// make new PID controler

	private final PIDController pidController = new PIDController(
			BalenceConstants.BALANCE_P,
			BalenceConstants.BALANCE_I,
			BalenceConstants.BALANCE_D);

	/**
	 * Moves the robot to the center of the Charge Station and keeps it balanced, \\
	 * assumes that the robot is already in line with the Charge Station and just needs to move forward/backward
	 *
	 * @param drivetrain The drivetrain of the robot.
	 */
	public BalanceCommand(SwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);

	}

	// get destination position 
	@Override
	public void initialize() {
		// TODO: Figure out if center of mass should be added or subtracted
		pidController.setSetpoint(0);
		pidController.setTolerance(0.1);

	}

	// get robot position and calculate speed 
	@Override
	public void execute() {
		double speed = pidController.calculate(drivetrain.getGyro().getYaw(), 0);
		drivetrain.setSwerveModuleStates(new ChassisSpeeds(speed, 0, 0));
		SmartDashboard.putNumber("Balance Command", speed);

	}

	// // Called once the command ends or is interrupted.
	// @Override
	// public void end(boolean interrupted) {
	// }

	@Override
	public boolean isFinished() {
		return pidController.atSetpoint();
	}
}
