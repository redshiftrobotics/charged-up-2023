package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalenceConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class BalenceGryoCommand extends CommandBase {
	private final SwerveDrivetrain drivetrain;
	private final AHRS gyro;

	private final PIDController pidController = new PIDController(
			SwerveDriveConstants.VELOCITY_PID_P,
			SwerveDriveConstants.VELOCITY_PID_I,
			SwerveDriveConstants.VELOCITY_PID_D);

	public BalenceGryoCommand(SwerveDrivetrain drivetrain, AHRS gyro) {
		this.drivetrain = drivetrain;
		this.gyro = gyro;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		final double pitch = gyro.getPitch();

		if (Math.abs(pitch) > BalenceConstants.BALENCE_TOLERANCE)
			return;

		final double speed = (pitch / 180) * BalenceConstants.SPEED_MULTPLIER;

		drivetrain.setSwerveModuleStates(new ChassisSpeeds(speed, 0, 0));

	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
