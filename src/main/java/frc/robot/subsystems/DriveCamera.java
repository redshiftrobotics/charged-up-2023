package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveCamera extends SubsystemBase {
	// private final UsbCamera cam;

	public DriveCamera() {
		// this.cam = cam;
		CameraServer.startAutomaticCapture();
	}
}
