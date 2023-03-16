package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//makes up the claw subsystem & claw functions as telling the claw to move and connecting it to its motor
public class Claw extends SubsystemBase {

	private final CANSparkMax clawMotor;
	private double clawDirection;

	public Claw(int clawMotorId) {
		clawMotor = new CANSparkMax(clawMotorId, MotorType.kBrushless);
	}

	//Setting the direction of the claw, called using the commands in RobotContainer through SetClawDirection
	public void setDirection(double desiredClawDirection) {
		clawDirection = desiredClawDirection;
	}

	//Periodically setting the claw according to the claw direction
	@Override
	public void periodic() {
		clawMotor.set(clawDirection);
	}

}