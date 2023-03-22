package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//makes up the claw subsystem & claw functions as telling the claw to move and connecting it to its motor
public class JakeClaw extends SubsystemBase {

	private final VictorSP clawMotor1;
	private final VictorSP clawMotor2;
	private final MotorControllerGroup clawMotors;
	private double clawSpeed;

	public JakeClaw(int clawMotor1Id, int clawMotor2Id) {
		clawMotor1 = new VictorSP(clawMotor1Id);
		clawMotor2 = new VictorSP(clawMotor2Id);
		clawMotors = new MotorControllerGroup(clawMotor1, clawMotor2);
	}

	//Setting the direction of the claw, called using the commands in RobotContainer through SetClawDirection
	public void setDirection(double desiredSpeed) {
		clawSpeed = desiredSpeed;
	}

	//Periodically setting the claw according to the claw direction
	@Override
	public void periodic() {
		clawMotors.set(clawSpeed);
	}

}