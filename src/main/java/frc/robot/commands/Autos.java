// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrivetrain;
import autos.testAuto

public final class Autos {
	/** Example static factory for an autonomous command. */
	// public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
	// 	return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	// }

	public static SequentialCommandGroup auto1(SwerveDrivetrain drivetrain) {
		return new TestAuto(drivetrain);
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
