package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//makes up the claw subsystem & claw functions as telling the claw to move and connecting it to its motor
public class JakeClaw extends SubsystemBase {

	private final VictorSPX clawMotor1;
	private final VictorSPX clawMotor2;
	private double clawSpeed;

	public JakeClaw(int clawMotor1Id, int clawMotor2Id) {
		clawMotor1 = new VictorSPX(clawMotor1Id);
		clawMotor2 = new VictorSPX(clawMotor2Id);
	}

	//Setting the direction of the claw, called using the commands in RobotContainer through SetClawDirection
	public void setDirection(double desiredSpeed) {
		clawSpeed = desiredSpeed;
	}

	//Periodically setting the claw according to the claw direction
	@Override
	public void periodic() {
		clawMotor1.set(VictorSPXControlMode.PercentOutput, clawSpeed);
		clawMotor2.set(VictorSPXControlMode.PercentOutput, clawSpeed);
		SmartDashboard.putNumber("Jake claw speed", clawSpeed);
	}

}