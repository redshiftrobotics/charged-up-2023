package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmManager extends SubsystemBase {
	public final BottomArm bottomArm;
	public final TopArm topArm;

	// Getting the bottom and top arm for their respective codes
	public ArmManager(BottomArm bottomArm, TopArm topArm) {
		this.bottomArm = bottomArm;
		this.topArm = topArm;
	}

	// Setting the degree of both arms for their mathamatics and their respective functions
	public void setBothArmDegree(double desiredBottomArmDegree, double desiredTopArmDegree) {
		bottomArm.setDegree(desiredBottomArmDegree);
		topArm.setDegree(desiredTopArmDegree);
	}

	// Getting the degree of both arms for rotations of input, reading, and execution
	public double[] getBothArmDegree() {
		double[] arr = { bottomArm.getEncoderRotation().getDegrees(), topArm.getEncoderRotation().getDegrees() };
		return arr;
	}

	public double[] getBothArmSetDegree() {
		double[] arr = { bottomArm.getTarget().getDegrees(), topArm.getTarget().getDegrees() };
		return arr;
	}
}