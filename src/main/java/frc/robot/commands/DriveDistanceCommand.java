// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveDriveConstants;

public class DriveDistanceCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;
	private final Translation2d driveDistance;
	private Translation2d distance;
	private Translation2d initialPosition;
	private final boolean isFieldRelative;
	private Translation2d position;
	private final PIDController pidController = new PIDController(
			SwerveDriveConstants.ROBOT_VELOCITY_PID_P,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_I,
			SwerveDriveConstants.ROBOT_VELOCITY_PID_D);

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public DriveDistanceCommand(SwerveDrivetrain drivetrain, Translation2d driveDistance, boolean isFieldRelative) {
		this.drivetrain = drivetrain;
		this.driveDistance = driveDistance;
		initialPosition = drivetrain.getRobotPosition().getTranslation();
		position = initialPosition;
		distance = driveDistance;
		this.isFieldRelative = isFieldRelative;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.setFieldRelative(isFieldRelative);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		position = drivetrain.getRobotPosition().getTranslation();
		distance = driveDistance.minus(position);
		Translation2d speed = new Translation2d(pidController.calculate(distance.getNorm()), distance.getAngle());
		drivetrain.setSwerveModuleStates(new ChassisSpeeds(speed.getX(), speed.getY(), 0));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.setSwerveModuleStates(new ChassisSpeeds());
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
