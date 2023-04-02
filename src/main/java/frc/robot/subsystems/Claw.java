package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Claw extends SubsystemBase {
	private final CANSparkMax motor1 = new CANSparkMax(ClawConstants.MOTOR_1_ID, MotorType.kBrushless);
	private final CANSparkMax motor2 = new CANSparkMax(ClawConstants.MOTOR_2_ID, MotorType.kBrushless);

	private final MotorControllerGroup motors = new MotorControllerGroup(motor1, motor2);

	public Claw() {
		motor1.setIdleMode(IdleMode.kBrake);
		motor2.setIdleMode(IdleMode.kBrake);
	}

	public void stop() {
		motors.set(0);
	}

	public void grabItem() {
		motors.set(-ClawConstants.MOTOR_SPEED);
	}

	public void ejectItem() {
		motors.set(0.5 * ClawConstants.MOTOR_SPEED);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Claw Motor 1", motor1.getOutputCurrent());
		SmartDashboard.putNumber("Claw Motor 2", motor2.getOutputCurrent());
	}
}
