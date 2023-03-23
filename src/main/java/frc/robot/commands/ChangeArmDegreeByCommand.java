package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmManager;

public class ChangeArmDegreeByCommand extends CommandBase {
	private final ArmManager arm;
	private final double bottomArmDegree;
	private final double topArmDegree;

	public ChangeArmDegreeByCommand(ArmManager arm, double bottomArmDegree, double topArmDegree) {
		this.arm = arm;
		this.bottomArmDegree = bottomArmDegree;
		this.topArmDegree = topArmDegree;
		addRequirements(this.arm);
	}

	@Override
	public void execute() {
		double[] currentAngles = arm.getBothArmSetDegree();
		double top = loopDegrees(topArmDegree + currentAngles[1]);
		double bottom = loopDegrees(currentAngles[0] + bottomArmDegree);
		arm.setBothArmDegree(bottom, top);
	}

	//If degrees is over 360 then minus it by 360
	//If degree is under 0 add 360 to it
	private double loopDegrees(double degree) {
		if (degree > 360) {
			return degree - 360;
		}
		if (degree < 0) {
			return degree + 360;
		}
		return degree;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
