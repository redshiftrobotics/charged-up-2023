package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmManager extends SubsystemBase {
	public final Arm bottomArm;
	public final Arm topArm;

	// Getting the bottom and top arm
	public ArmManager(Arm topArm, Arm bottomArm) {
		this.topArm = new Arm(
				ArmConstants.BOTTOM_ARM_MOTOR_ID,
				ArmConstants.BOTTOM_ARM_ENCODER_ID,
				ArmConstants.BOTTOM_ARM_MIN_DEGREE,
				ArmConstants.BOTTOM_ARM_MAX_DEGREE);

		this.bottomArm = new Arm(
				ArmConstants.TOP_ARM_MOTOR_ID,
				ArmConstants.TOP_ARM_ENCODER_ID,
				ArmConstants.TOP_ARM_MIN_DEGREE,
				ArmConstants.TOP_ARM_MAX_DEGREE);
	}
}