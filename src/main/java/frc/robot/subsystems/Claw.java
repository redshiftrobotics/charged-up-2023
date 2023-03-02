package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//makes up the claw subsystem & functions
public class Claw extends SubsystemBase {

	private final CANSparkMax clawMotor;
	private double clawDirection;

	public Claw(int clawMotorId) {
		clawMotor = new CANSparkMax(clawMotorId, MotorType.kBrushless);
	}

	public void setDirection(double desiredClawDirection) {
		clawDirection = desiredClawDirection;
	}

	@Override
	public void periodic() {
		clawMotor.set(clawDirection);
	}

}