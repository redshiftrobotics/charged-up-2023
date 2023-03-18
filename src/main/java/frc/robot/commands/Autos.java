// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmManager;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.Autos;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Autos {
	/** Example static factory for an autonomous command. */
	// public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
	// 	return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	// }

	// 
	public static SequentialCommandGroup basicAuto(SwerveDrivetrain drivetrain, ArmManager armManager) {
		// starts in front of bottom Grid node, aligned with it and pushed up against barriers	
		Translation2d START_POS = FieldConstants.BLUE_GRID_NODE_1.plus(
				new Translation2d(RobotConstants.WIDTH / 2, 0));

		// hideous abombination
		return new SequentialCommandGroup(
				// place cube on 3rd level of Grid
				RobotContainer.armScoreThreeCommand,

				// turn around and drive to staging point 1
				new ParallelCommandGroup(
						RobotContainer.armDriveCommand,
						new RotateAndDriveSimultaneousCommand(
								drivetrain,
								new Rotation2d(180), true,
								FieldConstants.BLUE_STAGING_MARK_1.minus(START_POS), true)),

				// grab new game object
				RobotContainer.armIntakeLowCommand,

				// drive back to Grid node 2
				new ParallelCommandGroup(
						RobotContainer.armDriveCommand,
						new DriveDistanceCommand(drivetrain,
								FieldConstants.BLUE_GRID_NODE_2.plus(new Translation2d(RobotConstants.LENGTH / 2, 0)),
								true)),

				// score at level 3 at node 2
				RobotContainer.armScoreThreeCommand);

	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
