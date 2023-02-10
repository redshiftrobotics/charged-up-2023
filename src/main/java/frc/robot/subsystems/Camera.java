// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.VideoDisplayConstants;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import org.opencv.core.CvType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class Camera extends SubsystemBase {
	private final UsbCamera camera;
	private final CvSink cvSink;

	// create mats here because they are quite exspensive to make
	private Mat mat;
	private final Mat grayMat;

	private final AprilTagDetector aprilTagDetector;
	private final AprilTagPoseEstimator aprilTagPoseEstimator;

	private AprilTagDetection[] detectedAprilTags;

	private final CvSource outputStream;

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

		outputStream = CameraServer.putVideo("AprilTags", CameraConstants.CAMERA_RESOLUTION_WIDTH,
				CameraConstants.CAMERA_RESOLUTION_HEIGHT);

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

	/** @return actual april tag from AprilTagDetections with specified ID. If it is not found it returns null. 
	 * NOTE: The tag and the tagId are different. The tagId is just an int while tag has more data and is what's found by AprilTagDetector.*/
	public AprilTagDetection getDetectedAprilTagById(int tagId) {
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
	 * @return 3d pose of tag. Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 */
	public Transform3d estimateTagPose(AprilTagDetection tag) {
		return aprilTagPoseEstimator.estimate(tag);
	}

	public Translation2d makePose2d(Transform3d pose) {
		return new Translation2d(pose.getZ(), pose.getX());
	}

	/** returns distance to pose in mm
	 * @param pose3d a 3d pose
	 * @return distance to tag in mm
	 */
	public double getDistance(Transform3d pose3d) {
		return Math.sqrt(
				Math.pow(pose3d.getX(), 2) + Math.pow(pose3d.getY(), 2) + Math.pow(pose3d.getZ(), 2));
	}

	/** Outlines and adds helpful data to diplay  */
	public void decorateTagInImage(AprilTagDetection tag, Transform3d tagPose3d, Mat mat) {

		final double distanceMM = getDistance(tagPose3d);

		final double distanceInch = distanceMM / 25.4;

		final Point pt0 = new Point(tag.getCornerX(0), tag.getCornerY(0));
		final Point pt1 = new Point(tag.getCornerX(1), tag.getCornerY(1));
		final Point pt2 = new Point(tag.getCornerX(2), tag.getCornerY(2));
		final Point pt3 = new Point(tag.getCornerX(3), tag.getCornerY(3));
		final Point center = new Point(tag.getCenterX(), tag.getCenterY());

		Imgproc.line(mat, pt0, pt1, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt1, pt2, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt2, pt3, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt3, pt0, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);

		final String tagIdMessage = Integer.toString(tag.getId());

		// final Size tagIdMessageSize = Imgproc.getTextSize();

		Imgproc.putText(mat, tagIdMessage, center, Imgproc.FONT_HERSHEY_DUPLEX,
				3, VideoDisplayConstants.TEXT_COLOR, 3);

		Imgproc.putText(mat, String.format("%.2f", distanceInch), pt0, Imgproc.FONT_HERSHEY_DUPLEX,
				1, VideoDisplayConstants.TEXT_COLOR, 1);
		// Imgproc.putText(mat, String.format("%.2f", tagPose3d.getRotation().getY()), pt0, Imgproc.FONT_HERSHEY_DUPLEX,
		// 		1, VideoDisplayConstants.TEXT_COLOR, 1);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// cvSink.grabFrame stores the camera frame in mat. It returns 0 on error.
		final long timeToFrame = cvSink.grabFrame(mat);

		if (timeToFrame != 0) {
			// Convert mat to gray scale and store it in grayMat
			Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

			// Detects all AprilTags in grayMat and store them
			detectedAprilTags = aprilTagDetector.detect(grayMat);

			// Loops through each april tag detected, once one is
			for (AprilTagDetection tag : detectedAprilTags) {
				Transform3d pose3d = estimateTagPose(tag); // takes 2-4 milliseconds
				// long startEstimationTime = System.currentTimeMillis();
				decorateTagInImage(tag, pose3d, mat); // takes about 1 millisecond or less
				// System.out.printf("Estimation Time: %s", System.currentTimeMillis() - startEstimationTime);
			}
			outputStream.putFrame(mat);
		}
	}
}