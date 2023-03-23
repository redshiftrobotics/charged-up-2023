package frc.robot.subsystems;

import java.lang.reflect.Constructor;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveCamera extends SubsystemBase {
	// private final UsbCamera cam;

	public DriveCamera() {
		// this.cam = cam;
		CameraServer.startAutomaticCapture();
	}
}
