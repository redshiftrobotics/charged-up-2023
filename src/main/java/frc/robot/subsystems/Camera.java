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
import org.opencv.core.Size;

import java.util.Arrays;

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
	private AprilTagDetection[] detectedAprilTagsValid;

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

		// ---------- Set up aprilTagDetector ----------

		aprilTagDetector = new AprilTagDetector();

		final AprilTagDetector.Config detectorConfig = aprilTagDetector.getConfig();
		// set config, see CameraConstants for comments what each setting does
		detectorConfig.quadDecimate = CameraConstants.QUAD_DECIMATE;
		detectorConfig.quadSigma = CameraConstants.QUAD_SIGMA;
		detectorConfig.refineEdges = CameraConstants.REFINE_EDGES;
		detectorConfig.decodeSharpening = CameraConstants.DECODE_SHARPENING;
		aprilTagDetector.setConfig(detectorConfig);

		final AprilTagDetector.QuadThresholdParameters quadThresholdParameters = aprilTagDetector
				.getQuadThresholdParameters();
		// set quad config, see CameraConstants for comments what each setting does
		quadThresholdParameters.minClusterPixels = CameraConstants.MIN_CLUSTER_PIXELS;
		quadThresholdParameters.criticalAngle = CameraConstants.CRITICAL_ANGLE;
		quadThresholdParameters.maxLineFitMSE = CameraConstants.MAX_LINE_FIT_MSE;
		quadThresholdParameters.minWhiteBlackDiff = CameraConstants.MIN_WHITE_BLACK_DIFF;
		quadThresholdParameters.deglitch = CameraConstants.DEGLITCH;
		aprilTagDetector.setQuadThresholdParameters(quadThresholdParameters);

		aprilTagDetector.addFamily(CameraConstants.TAG_FAMILY);

		// ---------- Set up aprilTagPoseEstimator ----------

		final AprilTagPoseEstimator.Config poseConfig = new AprilTagPoseEstimator.Config(
				CameraConstants.APRIL_TAG_SIZE_MM, CameraConstants.CAMERA_FOCAL_LENGTH_X,
				CameraConstants.CAMERA_FOCAL_LENGTH_Y, CameraConstants.CAMERA_FOCAL_CENTER_X,
				CameraConstants.CAMERA_FOCAL_CENTER_Y);

		aprilTagPoseEstimator = new AprilTagPoseEstimator(poseConfig);
	}

	/** @return list of all AprilTagDetections found by camera */
	public AprilTagDetection[] getAllDetectedAprilTags() {
		return detectedAprilTags;
	}

	/** 
	 * valid tags are 
	 * @return list of all AprilTagDetections found by camera that meet the requirements of 
	 */
	public AprilTagDetection[] getDetectedAprilTags() {
		return detectedAprilTagsValid;
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

	/** gets position of tag relative to camera.
	 * @param tag a AprilTagDetection
	 * @return 3d pose of tag. Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 */
	public Transform3d getTagPose(AprilTagDetection tag) {
		// Translation3d https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png
		// Rotation3d Quaternion https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png
		return aprilTagPoseEstimator.estimate(tag);
	}

	/** converts position of tag relative to camera.
	 * @param pose3d a 3d pose of a tag
	 * @return 2d pose of a tag (ignores verticle). Translation2d(x: +foward, y: -right to +left)
	 */
	public Translation2d makePose2d(Transform3d pose3d) {
		return new Translation2d(pose3d.getZ(), pose3d.getX());
	}

	/** returns distance to pose in mm
	 * @param pose3d a 3d pose of tag
	 * @return distance to tag in mm
	 */
	public double getDistance(Transform3d pose3d) {
		return Math.sqrt(
				Math.pow(pose3d.getX(), 2) + Math.pow(pose3d.getY(), 2) + Math.pow(pose3d.getZ(), 2));
	}

	/** returns distance to pose in mm without verticle distance
	 * @param pose2d a 2d pose of tag (ignores verticle)
	 * @return distance to tag in mm
	 */
	public double getDistance(Translation2d pose2d) {
		return Math.sqrt(Math.pow(pose2d.getX(), 2) + Math.pow(pose2d.getY(), 2));
	}

	/** Outlines and adds helpful data to diplay about a tag in Mat */
	public void decorateTagInImage(Mat mat, AprilTagDetection tag, Transform3d tagPose3d) {
		// get distance to tag and convert it to feet

		// final double distanceMM = getDistance(makePose2d(tagPose3d));
		final double distanceMM = getDistance(tagPose3d);

		// create point for each corner of tag
		final Point pt0 = new Point(tag.getCornerX(0), tag.getCornerY(0));
		final Point pt1 = new Point(tag.getCornerX(1), tag.getCornerY(1));
		final Point pt2 = new Point(tag.getCornerX(2), tag.getCornerY(2));
		final Point pt3 = new Point(tag.getCornerX(3), tag.getCornerY(3));

		// Draw lines around box with corner points
		Imgproc.line(mat, pt0, pt1, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt1, pt2, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt2, pt3, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt3, pt0, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);

		// draw tag Id on tag
		addCenteredText(mat, Integer.toString(tag.getId()), 4, 4,
				new Point(tag.getCenterX(), tag.getCenterY()));

		// draw distance to tag under tag Id

		final double distanceFeet = distanceMM / 304.8;
		addCenteredText(mat, String.format("%.1f ft.", distanceFeet), 1, 2,
				new Point(tag.getCenterX(), tag.getCenterY() * 1.2));

		// final double distanceInch = distanceMM / 25.4;
		// addCenteredText(mat, String.format("%.2f", distanceInch), 1, 2,
		// 		new Point(tag.getCenterX(), tag.getCenterY() * 1.2));
	}

	/** add text centerd on point to mat */
	public void addCenteredText(Mat mat, String text, int fontScale, int thickness, Point org) {
		// get width and height of text
		final Size textSize = Imgproc.getTextSize(text, VideoDisplayConstants.FONT_TYPE, fontScale, thickness, null);

		// find point where the text goes if it were centered
		// Y is added because Point(0, 0) is top left of Mat
		final Point textCenter = new Point(org.x - (textSize.width / 2), org.y + (textSize.height / 2));

		// add text outline
		Imgproc.putText(mat, text, textCenter, VideoDisplayConstants.FONT_TYPE,
				fontScale, VideoDisplayConstants.WHITE, thickness + 2);
		// add text
		Imgproc.putText(mat, text, textCenter, VideoDisplayConstants.FONT_TYPE,
				fontScale, VideoDisplayConstants.TEXT_COLOR, thickness);

	}

	/** Check if tag should be consideded by checking if its ID is possible and if its decision margin is high enough  */
	public boolean isTagValid(AprilTagDetection tag) {
		return tag.getId() >= CameraConstants.MIN_TAG_NUMBER
				&& tag.getId() <= CameraConstants.MAX_TAG_NUMBER
				&& tag.getDecisionMargin() > CameraConstants.MIN_APRIL_TAG_DECISION_MARGIN;
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

			// Filters out invalid tags
			detectedAprilTagsValid = Arrays.stream(detectedAprilTags)
					.filter(this::isTagValid).toArray(AprilTagDetection[]::new);

			for (AprilTagDetection tag : getDetectedAprilTags()) {
				Transform3d pose3d = getTagPose(tag);
				decorateTagInImage(mat, tag, pose3d);
			}

			outputStream.putFrame(mat);
		}

	}
}