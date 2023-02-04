// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj2.command.CommandBase;

// TODO extends DriveDistanceCommand
public class GoToTagCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Camera camera;

	public GoToTagCommand(Camera camera) {
		this.camera = camera;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(camera);
	}
}
