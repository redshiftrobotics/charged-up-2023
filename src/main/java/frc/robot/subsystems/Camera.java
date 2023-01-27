// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.apriltag.AprilTagDetector.QuadThresholdParameters;

public class Camera extends SubsystemBase {
	private final UsbCamera camera;
	private final CvSink cvSink = CameraServer.getVideo();
	private final Mat mat = new Mat();
	private final Mat grayMat = new Mat();
	private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

	public Camera(int cameraID) {
		this.camera = CameraServer.startAutomaticCapture(cameraID);

		camera.setResolution(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT);

		Config config = aprilTagDetector.getConfig();
		config.quadSigma = 0.8f;
		aprilTagDetector.setConfig(config);

		QuadThresholdParameters quadThresholdParameters = aprilTagDetector.getQuadThresholdParameters();
		quadThresholdParameters.minClusterPixels = CameraConstants.MIN_CLUSTER_PIXELS;
		quadThresholdParameters.criticalAngle = CameraConstants.CRITICAL_ANGLE;
		quadThresholdParameters.maxLineFitMSE = CameraConstants.MAX_LINE_FIT_MSE;
		aprilTagDetector.setQuadThresholdParameters(quadThresholdParameters);

		aprilTagDetector.addFamily(CameraConstants.TAG_FAMILY);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
