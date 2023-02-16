// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.depricatedSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// spins wheels to intake cones and cubes
public class Intake extends SubsystemBase {
	private final CANSparkMax motorTop;
	private final CANSparkMax motorBottom;
	private boolean isOn = false;
	private int direction = 1;

	/** initializes motors  */
	public Intake(int motorTopID, int motorBottomID) {
		this.motorTop = new CANSparkMax(motorTopID, MotorType.kBrushless);
		this.motorBottom = new CANSparkMax(motorBottomID, MotorType.kBrushless);
	}

	// turn intake on/offn
	public void toggle() {
		//TODO: figure out which motor needs to be negative
		if (isOn) {
			motorTop.set(0);
			motorBottom.set(0);
			isOn = false;
		} else {
			motorTop.set(direction * IntakeConstants.INTAKE_MOTOR_SPEED);
			motorBottom.set(-direction * IntakeConstants.INTAKE_MOTOR_SPEED);
			isOn = true;
		}
	}

	//changes the direction of the intake wheels 
	public void toggleDirection() {
		direction *= -1;
	}
	// turns intake motors on 
	// public void turnOn(){
	// 	motorTop.set(IntakeConstants.INTAKE_MOTOR_SPEED);
	// 	motorBottom.set(IntakeConstants.INTAKE_MOTOR_SPEED);
	// }

	// // turns intake motors off
	// public void turnOff(){
	// 	motorTop.set(0);
	// 	motorBottom.set(0);
	// }

	// @Override
	// public void periodic() {
	// 	// This method will be called once per scheduler run
	// }

}
