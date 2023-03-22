package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.GroupLayout.Group;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class SpringClaw extends SubsystemBase {

	public double clawSpeed = 0;

	public final VictorSPX motor1;
	public final VictorSPX motor2;
	// public final GroupMotorControllers motorGroup;

	public SpringClaw(int clawMotor1Id, int clawMotor2Id) {
		motor1 = new VictorSPX(clawMotor1Id);
		motor2 = new VictorSPX(clawMotor2Id);
	}

	public void setSpeed(double speed) {
		clawSpeed = speed;
	}

	@Override
	public void periodic() {
		motor1.set(ControlMode.Velocity, clawSpeed);
		motor2.set(ControlMode.Velocity, clawSpeed);
	}
}
