// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ToggleFieldRelativeCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SwerveDrivetrain drivetrain;

	/**
	 * Creates a new ToggleFieldRelativeCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ToggleFieldRelativeCommand(SwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.toggleFieldRelative();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
