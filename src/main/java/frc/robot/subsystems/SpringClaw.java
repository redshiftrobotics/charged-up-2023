package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpringClaw extends SubsystemBase {

	public double clawSpeed = 0;

	public VictorSP motor;

	public SpringClaw(int clawMotorId) {
		motor = new VictorSP(clawMotorId);
	}

	public void setSpeed(double speed) {
		clawSpeed = speed;
	}

	@Override
	public void periodic() {
		motor.set(clawSpeed);
	}
}
