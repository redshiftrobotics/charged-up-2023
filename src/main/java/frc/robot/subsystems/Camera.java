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
	private final CvSink cvSink;

	private Mat mat;
	private final Mat grayMat;

	private final AprilTagDetector aprilTagDetector;

	private AprilTagDetection[] detectedAprilTags;

	/** Constructor for Camera.
	 * Creates a UsbCamera object with CameraServer and sets its resolution.
	 * Configures a aprilTagDetector 
	 * @param cameraID ID of the camera
	 */
	public Camera(int cameraID) {

		// create mat with color (8 bits, 3 channels)
		try {

			mat = new Mat(
					CameraConstants.CAMERA_RESOLUTION_WIDTH,
					CameraConstants.CAMERA_RESOLUTION_HEIGHT,
					CvType.CV_8UC3);
		} catch (Exception e) {
			System.out.println("error !!!");
			System.out.println(e);
			mat = null;
			// TODO: handle exception
		}

		// create black and white mat (8 bits, 1 channel)
		grayMat = new Mat(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC1);

		camera = CameraServer.startAutomaticCapture(cameraID);

		cvSink = CameraServer.getVideo();

		camera.setResolution(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT);

		// set up aprilTagDetector
		aprilTagDetector = new AprilTagDetector();

		final Config config = aprilTagDetector.getConfig();
		// set config, see CameraConstants for comments what each setting does
		config.quadSigma = CameraConstants.QUAD_SIGMA;
		aprilTagDetector.setConfig(config);

		final QuadThresholdParameters quadThresholdParameters = aprilTagDetector.getQuadThresholdParameters();
		// set quad config, see CameraConstants for comments what each setting does
		quadThresholdParameters.minClusterPixels = CameraConstants.MIN_CLUSTER_PIXELS;
		quadThresholdParameters.criticalAngle = CameraConstants.CRITICAL_ANGLE;
		quadThresholdParameters.maxLineFitMSE = CameraConstants.MAX_LINE_FIT_MSE;
		aprilTagDetector.setQuadThresholdParameters(quadThresholdParameters);

		aprilTagDetector.addFamily(CameraConstants.TAG_FAMILY);
	}

	/** get distance to an object from the camera
	 * @param knownSizeInchs the known width/height of the object you are tring to detect 
	 * @param size the width/height of the object you have found
	 * @return distance to object in inches
	*/
	public static double getDistanceToObjectFromCamera(double knownSizeInchs, double size) {
		return (knownSizeInchs * CameraConstants.CAMERA_FOCAL_LENGTH) / size;
	}

	public static double getDistanceToAprilTag(AprilTagDetection aprilTag) {
		final double distance1 = getDistanceToObjectFromCamera(CameraConstants.APRIL_TAG_SIZE_INCHS,
				aprilTag.getCornerX(1) - aprilTag.getCornerX(0));
		final double distance2 = getDistanceToObjectFromCamera(CameraConstants.APRIL_TAG_SIZE_INCHS,
				aprilTag.getCornerY(1) - aprilTag.getCornerY(0));

		final double avgDistance = (distance1 + distance2) / 2;

		System.out.println(String.format("w: %s, h: %s, avg: %s", distance1, distance2, avgDistance));

		return avgDistance;
	}

	/** @return list of all AprilTagDetections found by camera */
	public AprilTagDetection[] getDetectedAprilTags() {
		return detectedAprilTags;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// cvSink.grabFrame stores the camera frame in mat. It returns 0 on error.
		final long timeToFrame = cvSink.grabFrame(mat);

		if (timeToFrame != 0) {
			// convert mat to gray scale and store it in grayMat
			Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

			// detects all AprilTags in grayMat and store them
			detectedAprilTags = aprilTagDetector.detect(grayMat);
		}
	}

}
