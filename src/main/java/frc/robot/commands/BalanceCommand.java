package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class BalanceCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;

	// TODO figure what constants to use 
	// make new PID controler

	private final PIDController pidController = new PIDController(
			SwerveDriveConstants.ROBOT_BALANCE_PID_P,
			SwerveDriveConstants.ROBOT_BALANCE_PID_I,
			SwerveDriveConstants.ROBOT_BALANCE_PID_D);

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
		pidController.setTolerance(SwerveDriveConstants.ROBOT_PITCH_TOLERANCE,
				SwerveDriveConstants.ROBOT_STOP_PITCH_TOLERANCE);

	}

	// get destination position 
	@Override
	public void initialize() {
		drivetrain.setFieldRelative(true);
	}

	// get robot position and calculate speed 
	@Override
	public void execute() {
		double speed = pidController.calculate(drivetrain.getRobotPitchRotation());
		drivetrain.setSwerveModuleStates(new ChassisSpeeds(0, speed, 0));
	}

	// // Called once the command ends or is interrupted.
	// @Override
	// public void end(boolean interrupted) {
	// }

	@Override
	public boolean isFinished() {
		return pidController.atSetpoint() &
				drivetrain.getVelocity().getNorm() < SwerveDriveConstants.ROBOT_STOP_VELOCITY_TOLERANCE;
	}
}
