// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;

import java.util.ArrayList;

import org.opencv.core.CvType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

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
				CameraConstants.APRIL_TAG_SIZE_MM, CameraConstants.CAMERA_FOCAL_LENGTH_X,
				CameraConstants.CAMERA_FOCAL_LENGTH_Y, CameraConstants.CAMERA_FOCAL_CENTER_X,
				CameraConstants.CAMERA_FOCAL_CENTER_Y);

		aprilTagPoseEstimator = new AprilTagPoseEstimator(poseConfig);
	}

	/** @return list of all AprilTagDetections found by camera */
	public AprilTagDetection[] getDetectedAprilTags() {
		return detectedAprilTags;
	}

	/** @return returns AprilTagDetections with specified. If it is not found it returns null */
	public AprilTagDetection getDetectedAprilTag(int tagId) {
		for (AprilTagDetection tag : getDetectedAprilTags()) {
			if (tag.getId() == tagId) {
				return tag;
			}
		}
		return null;
	}

	/** Returns position of tag relative to camera.
	 * Translation3d https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png
	 * Rotation3d Quaternion https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png
	 * @param tag a AprilTagDetection
	 * @return Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: distance), Rotation3d(Quaternion(...)))
	 */
	public Transform3d estimateTagPose(AprilTagDetection tag) {
		return aprilTagPoseEstimator.estimate(tag);
	}

	/** Returns position of tag relative to camera in 2d.
	 * @param tag a AprilTagDetection
	 * @return simple Translation2d towards april tag
	 */
	public Translation2d estimateTagPoseSimple(AprilTagDetection tag) {
		final Transform3d pose = estimateTagPose(tag);
		return new Translation2d(pose.getZ(), pose.getX());
	}

	public double getDistance(Transform3d transform) {
		return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2) + Math.pow(transform.getZ(), 2));
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
