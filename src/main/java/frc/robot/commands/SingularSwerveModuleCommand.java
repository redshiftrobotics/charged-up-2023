// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SingularSwerveModuleCommand extends CommandBase {
	// @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	// private final ExampleSubsystem subsystem;
	private final SwerveModule swerveModule;
	private final SwerveModuleState state;

	public SingularSwerveModuleCommand(SwerveModule swerveModule, double angle, double velocity) {
		this.swerveModule = swerveModule;
		state = new SwerveModuleState();
		state.angle = Rotation2d.fromRadians(angle);
		state.speedMetersPerSecond = velocity;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.swerveModule);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putBoolean("Command Initialize", true);
		swerveModule.setSwerveModuleState(state);
	}

	// // Called every time the scheduler runs while the command is scheduled.
	// @Override
	// public void execute() {
	// }

	// // Called once the command ends or is interrupted.
	// @Override
	// public void end(boolean interrupted) {
	// }

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
