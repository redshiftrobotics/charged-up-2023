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
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;

public final class Autos {
	/** Example static factory for an autonomous command. */
	// public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
	// 	return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	// }

	// 

	private static final int dir = Constants.TEAM == "red" ? -1 : 1;

	private static Translation2d findRelativeOffset(Translation2d start, Translation2d goal) {
		return goal.minus(start);
	}

	public static SequentialCommandGroup basicAuto(SwerveDrivetrain drivetrain, ArmManager armManager) {
		// starts in front of bottom Grid node, aligned with it and pushed up against barriers

		final Translation2d START_POS = FieldConstants.BLUE_GRID_NODE_1.plus(
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
								new Rotation2d(Math.PI), false,
								findRelativeOffset(FieldConstants.BLUE_STAGING_MARK_1, START_POS).times(dir), true)),

				// grab new game object
				RobotContainer.armIntakeLowCommand,

				// drive back to Grid node 2
				new ParallelCommandGroup(
						RobotContainer.armDriveCommand,
						new RotateAndDriveSimultaneousCommand(drivetrain, new Rotation2d(Math.PI), false,
								findRelativeOffset(
										FieldConstants.BLUE_STAGING_MARK_1.minus(RobotConstants.PICKUP_OFFSET),
										FieldConstants.BLUE_GRID_NODE_2.plus(RobotConstants.LENGTH_OFFSET)).times(dir),
								true)),

				// score at level 3 at node 2
				RobotContainer.armScoreThreeCommand,

				new DriveDistanceCommand(drivetrain,
						findRelativeOffset(
								// Left of the charge station by the distance from center to edge and half the length of robot
								FieldConstants.BLUE_CHARGE_STATION.minus(
										FieldConstants.CHARGE_STATION_RAMP_OFFSET.minus(RobotConstants.LENGTH_OFFSET)),
								FieldConstants.BLUE_GRID_NODE_2.plus(RobotConstants.LENGTH_OFFSET)).times(dir),
						true),

				new BalanceCommand(drivetrain)

		);
	}

	public static SequentialCommandGroup basicAutoNoBalance(SwerveDrivetrain drivetrain) {
		// starts in front of bottom Grid node, aligned with it and pushed up against barriers

		final Translation2d START_POS = FieldConstants.BLUE_GRID_NODE_1.plus(
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
								new Rotation2d(Math.PI), false,
								findRelativeOffset(FieldConstants.BLUE_STAGING_MARK_1, START_POS).times(dir), true)),

				// grab new game object
				RobotContainer.armIntakeLowCommand,

				// drive back to Grid node 2
				new ParallelCommandGroup(
						RobotContainer.armDriveCommand,
						new RotateAndDriveSimultaneousCommand(drivetrain, new Rotation2d(Math.PI), false,
								findRelativeOffset(
										FieldConstants.BLUE_STAGING_MARK_1.minus(RobotConstants.PICKUP_OFFSET),
										FieldConstants.BLUE_GRID_NODE_2.plus(RobotConstants.LENGTH_OFFSET)).times(dir),
								true)),

				// score at level 3 at node 2
				RobotContainer.armScoreThreeCommand);
	}

	public static SequentialCommandGroup noArmAutoBottom(SwerveDrivetrain drivetrain) {
		// This auto simply leaves the community and then balances on the Charge Station
		final Translation2d START_POS = FieldConstants.BLUE_GRID_NODE_1.plus(
				new Translation2d(RobotConstants.WIDTH / 2, 0));

		return new SequentialCommandGroup(
				new DriveDistanceCommand(drivetrain,
						findRelativeOffset(START_POS,
								FieldConstants.BLUE_STAGING_MARK_1.minus(new Translation2d(RobotConstants.WIDTH, 0)))
								.times(dir),
						true),
				new DriveDistanceCommand(drivetrain,
						findRelativeOffset(
								FieldConstants.BLUE_STAGING_MARK_1.minus(new Translation2d(RobotConstants.WIDTH, 0)),
								FieldConstants.BLUE_CHARGE_STATION.plus(
										FieldConstants.CHARGE_STATION_RAMP_OFFSET.plus(RobotConstants.LENGTH_OFFSET)))
								.times(dir),
						true),
				new BalanceCommand(drivetrain));

	}

	public static SequentialCommandGroup noArmAutoTop(SwerveDrivetrain drivetrain) {
		// This auto simply leaves the community and then balances on the Charge Station
		final Translation2d START_POS = FieldConstants.BLUE_GRID_NODE_8.plus(
				new Translation2d(RobotConstants.WIDTH / 2, 0));

		return new SequentialCommandGroup(
				new DriveDistanceCommand(drivetrain,
						findRelativeOffset(START_POS,
								FieldConstants.BLUE_STAGING_MARK_4.minus(new Translation2d(RobotConstants.WIDTH, 0)))
								.times(dir),
						true),
				new DriveDistanceCommand(drivetrain,
						findRelativeOffset(
								FieldConstants.BLUE_STAGING_MARK_4.minus(new Translation2d(RobotConstants.WIDTH, 0)),
								FieldConstants.BLUE_CHARGE_STATION.plus(
										FieldConstants.CHARGE_STATION_RAMP_OFFSET.plus(RobotConstants.LENGTH_OFFSET)))
								.times(dir),
						true),
				new BalanceCommand(drivetrain)

		);

	}

	public static SequentialCommandGroup visionBasedAuto(SwerveDrivetrain drivetrain, ArmManager armManager) {
		return new SequentialCommandGroup(
			RobotContainer.armScoreThreeCommand,

			GoToTagCommand.createGoToTagCommand(drivetrain, , new Transform2d(findRelativeOffset(FieldConstants.TAGS.get(8), FieldConstants.BLUE_STAGING_MARK_1.minus(RobotConstants.LENGTH_OFFSET)), new Rotation2d(Math.PI))),

		);
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
