// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.CvType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.apriltag.AprilTagDetector.QuadThresholdParameters;

public class Camera extends SubsystemBase {
	private final UsbCamera camera;
	private final CvSink cvSink = CameraServer.getVideo();

	private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

	private final Mat mat = new Mat(CameraConstants.CAMERA_RESOLUTION_WIDTH, CameraConstants.CAMERA_RESOLUTION_HEIGHT,
			CvType.CV_8UC3);
	private final Mat grayMat = new Mat(CameraConstants.CAMERA_RESOLUTION_WIDTH,
			CameraConstants.CAMERA_RESOLUTION_HEIGHT, CvType.CV_8UC1);

	public Camera(int cameraID) {
		this.camera = CameraServer.startAutomaticCapture(cameraID);

		camera.setResolution(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT);

		Config config = aprilTagDetector.getConfig();
		config.quadSigma = CameraConstants.QUAD_SIGMA;
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
		if (cvSink.grabFrame(mat) != 0) {
			Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

			final AprilTagDetection[] results = aprilTagDetector.detect(grayMat);

			for (final AprilTagDetection result : results) {
				System.out.println(result);
			}
		}
	}

}
