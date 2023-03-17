// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.Autos;

public final class Autos {
	/** Example static factory for an autonomous command. */
	// public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
	// 	return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	// }

	// 
	public static SequentialCommandGroup auto1(SwerveDrivetrain drivetrain) {
		// starts in front of april tag 2 (8) and facing it
		return new SequentialCommandGroup(
		/* place cube on 3rd level of Grid
		
		turn around and drive to staging point 1
			- Drive to offset from april tag (not very accurate)
		
		grab new game object
		
		drive back to community
		
		*/

		);
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
