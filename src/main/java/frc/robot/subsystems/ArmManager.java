package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmManager extends SubsystemBase {
	public final Arm bottomArm;
	public final Arm topArm;

	// Getting the bottom and top arm
	public ArmManager(Arm bottomArm, Arm topArm) {
		this.bottomArm = bottomArm;
		this.topArm = topArm;
	}

	// Setting the degree of both arms
	public void setBothArmDegree(double desiredBottomArmDegree, double desiredTopArmDegree) {
		bottomArm.setDegree(desiredBottomArmDegree);
		topArm.setDegree(desiredTopArmDegree);
	}

	// Getting the degree of both arms
	public String getBothArmDegree() {
		return bottomArm.getEncoderRotation() + ", " + topArm.getEncoderRotation();
	}
}