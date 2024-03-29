// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.depricatedCommands;

import frc.robot.subsystems.depricatedSubsystem.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ToggleIntakeDirectionCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Intake intake;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ToggleIntakeDirectionCommand(Intake intake) {
		this.intake = intake;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intake.toggleDirection();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
