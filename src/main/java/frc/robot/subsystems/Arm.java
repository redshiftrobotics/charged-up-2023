// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Arm extends SubsystemBase {
// 	private final CANSparkMax armMotor;
// 	private final RelativeEncoder armEncoder;
// 	private final PIDController armPIDController;

// 	//definds the arm and its berriors
// 	public Arm(int armMotorId, int armEncoderId) {
// 		armMotor = new CANSparkMax(armMotorId, MotorType.kBrushless);
// 		armEncoder = armMotor.getEncoder();
// 		//armPIDController = new PIDController();

// 	}

// 	public double getEncoderRotation() {
// 		return 0;
// 	}

// 	//Oh no
// 	@Override
// 	public void periodic() {

// 	}

// 	//   _______     _
// 	//  /  ____  \  | |                _
// 	//  | /    \ |  | |          |_|
// 	//  | |    | |  | |  ___      _    _____
// 	//  | |    | |  | |/ ___ \   | |  /  _  \
// 	//  | |    | |  |  /    \ |  | |  | / \ |
// 	//  | \____/ |  | |     | |  | |  | \_/ |
// 	//  \________/  |_|     |_|  |_|  \_____/
// }
