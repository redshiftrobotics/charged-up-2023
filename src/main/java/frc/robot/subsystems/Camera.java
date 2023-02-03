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
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera extends SubsystemBase {
	private final UsbCamera camera;
	private final CvSink cvSink;

	private Mat mat;
	private final Mat grayMat;

	private final AprilTagDetector aprilTagDetector;
	private final AprilTagPoseEstimator aprilTagPoseEstimator;

	private AprilTagDetection[] detectedAprilTags;

	/** Constructor for Camera.
	 * Creates a UsbCamera object with CameraServer and sets its resolution.
	 * Creates and configures a AprilTagDetector and AprilTagPoseEstimator
	 * @param cameraID ID of the camera
	 */
	public Camera(int cameraID) {
		// -------------- Set up Camera --------------
		camera = CameraServer.startAutomaticCapture(cameraID);

		camera.setResolution(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT);

		cvSink = CameraServer.getVideo();

		// -------------- Set up Mats --------------

		// create mat with color (8 bits, 3 channels)
		mat = new Mat(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC3);

		// create black and white mat (8 bits, 1 channel)
		grayMat = new Mat(
				CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC1);

		// -------------- Set up aprilTagDetector --------------

		aprilTagDetector = new AprilTagDetector();

		final AprilTagDetector.Config detectorConfig = aprilTagDetector.getConfig();
		// set config, see CameraConstants for comments what each setting does
		detectorConfig.quadSigma = CameraConstants.QUAD_SIGMA;
		aprilTagDetector.setConfig(detectorConfig);

		final AprilTagDetector.QuadThresholdParameters quadThresholdParameters = aprilTagDetector
				.getQuadThresholdParameters();
		// set quad config, see CameraConstants for comments what each setting does
		quadThresholdParameters.minClusterPixels = CameraConstants.MIN_CLUSTER_PIXELS;
		quadThresholdParameters.criticalAngle = CameraConstants.CRITICAL_ANGLE;
		quadThresholdParameters.maxLineFitMSE = CameraConstants.MAX_LINE_FIT_MSE;
		aprilTagDetector.setQuadThresholdParameters(quadThresholdParameters);

		aprilTagDetector.addFamily(CameraConstants.TAG_FAMILY);

		// -------------- Set up aprilTagPoseEstimator --------------

		final AprilTagPoseEstimator.Config poseConfig = new AprilTagPoseEstimator.Config(
				CameraConstants.APRIL_TAG_SIZE_MM, CameraConstants.CAMERA_FOCAL_CENTER_X,
				CameraConstants.CAMERA_FOCAL_CENTER_Y, CameraConstants.CAMERA_FOCAL_CENTER_X,
				CameraConstants.CAMERA_FOCAL_CENTER_Y);

		aprilTagPoseEstimator = new AprilTagPoseEstimator(poseConfig);
	}

	public Transform3d estimateTagPose(AprilTagDetection tag) {
		return aprilTagPoseEstimator.estimate(tag);
	}

	public Transform3d estimateTagPoseHomography(AprilTagDetection tag) {
		return aprilTagPoseEstimator.estimateHomography(tag);
	}

	/** @return list of all AprilTagDetections found by camera */
	public AprilTagDetection[] getDetectedAprilTags() {
		return detectedAprilTags;
	}

	public double getDistance(Transform3d transfrom) {
		return Math.sqrt(Math.pow(transfrom.getX(), 2) + Math.pow(transfrom.getY(), 2) + Math.pow(transfrom.getY(), 2));
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
			for (AprilTagDetection tag : detectedAprilTags) {
				if (tag.getId() == 1) {
					System.out.println(tag);
					System.out.println();

					final Transform3d tagPose = estimateTagPose(tag);
					System.out.println(tagPose);
					final double distancePose = getDistance(tagPose);
					System.out.println(String.format("distance: %s (%s inchs)", distancePose, distancePose / 25.4));
					System.out.println();

					final Transform3d tagPoseHomography = estimateTagPoseHomography(tag);
					System.out.println(tagPoseHomography);
					final double distancePoseHomography = getDistance(tagPose);
					System.out.println(String.format("distance: %s (%s inchs)", distancePoseHomography,
							distancePoseHomography / 25.4));
					System.out.println();
				}
			}
		}
	}

}
