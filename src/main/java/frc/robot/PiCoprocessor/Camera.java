// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.PiCoprocessor;

import frc.robot.PiCoprocessor.Constants.AprilTagValueConstants;
import frc.robot.PiCoprocessor.Constants.DetectionConfigConstants;
import frc.robot.PiCoprocessor.Constants.PhysicalCameraConstants;
import frc.robot.PiCoprocessor.Constants.VideoDisplayConstants;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

import java.util.HashMap;

import org.opencv.core.CvType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
	private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
	private final NetworkTable table;

	public static void main(String[] args) {
		final Camera camera = new Camera(PhysicalCameraConstants.CAMERA_ID);

		while (true) {
			camera.periodic();
		}
	}

	private final UsbCamera camera;
	private final CvSink cvSink;

	// create mats here because they are quite exspensive to make
	private final Mat mat;
	private final Mat grayMat;

	private final AprilTagDetector aprilTagDetector;
	private final AprilTagPoseEstimator aprilTagPoseEstimator;

	private final CvSource outputStream;

	/** Constructor for Camera.
	 * Creates a UsbCamera object with CameraServer and sets its resolution.
	 * Creates and configures a AprilTagDetector and AprilTagPoseEstimator
	 * @param cameraID ID of the camera
	 */
	public Camera(int cameraID) {
		inst.startClient4("localpiclient");
		inst.setServerTeam(8032);
		inst.startDSClient();
		inst.setServer("localhost");

		table = inst.getTable(String.format("camera-%s-tags", cameraID));

		// -------------- Set up Camera --------------
		camera = CameraServer.startAutomaticCapture(cameraID);

		camera.setResolution(
				PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT);

		cvSink = CameraServer.getVideo();

		outputStream = CameraServer.putVideo("AprilTags", PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT);

		// -------------- Set up Mats --------------

		// create mat with color (8 bits, 3 channels)
		mat = new Mat(
				PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC3);

		// create black and white mat (8 bits, 1 channel)
		grayMat = new Mat(
				PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC1);

		// ---------- Set up aprilTagDetector ----------

		aprilTagDetector = new AprilTagDetector();

		final AprilTagDetector.Config detectorConfig = aprilTagDetector.getConfig();
		// set config, see CameraConstants for comments what each setting does
		detectorConfig.quadDecimate = DetectionConfigConstants.QUAD_DECIMATE;
		detectorConfig.quadSigma = DetectionConfigConstants.QUAD_SIGMA;
		detectorConfig.refineEdges = DetectionConfigConstants.REFINE_EDGES;
		detectorConfig.decodeSharpening = DetectionConfigConstants.DECODE_SHARPENING;
		aprilTagDetector.setConfig(detectorConfig);

		final AprilTagDetector.QuadThresholdParameters quadThresholdParameters = aprilTagDetector
				.getQuadThresholdParameters();
		// set quad config, see DetectionConfigConstants for comments what each setting does
		quadThresholdParameters.minClusterPixels = DetectionConfigConstants.MIN_CLUSTER_PIXELS;
		quadThresholdParameters.criticalAngle = DetectionConfigConstants.CRITICAL_ANGLE;
		quadThresholdParameters.maxLineFitMSE = DetectionConfigConstants.MAX_LINE_FIT_MSE;
		quadThresholdParameters.minWhiteBlackDiff = DetectionConfigConstants.MIN_WHITE_BLACK_DIFF;
		quadThresholdParameters.deglitch = DetectionConfigConstants.DEGLITCH;
		aprilTagDetector.setQuadThresholdParameters(quadThresholdParameters);

		aprilTagDetector.addFamily(DetectionConfigConstants.TAG_FAMILY);

		// ---------- Set up aprilTagPoseEstimator ----------

		final AprilTagPoseEstimator.Config poseConfig = new AprilTagPoseEstimator.Config(
				AprilTagValueConstants.APRIL_TAG_SIZE_MM, PhysicalCameraConstants.CAMERA_FOCAL_LENGTH_X,
				PhysicalCameraConstants.CAMERA_FOCAL_LENGTH_Y, PhysicalCameraConstants.CAMERA_FOCAL_CENTER_X,
				PhysicalCameraConstants.CAMERA_FOCAL_CENTER_Y);

		aprilTagPoseEstimator = new AprilTagPoseEstimator(poseConfig);
	}

	/** gets position of tag relative to camera.
	 * A 3d pose of a tag is a Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @param tag a AprilTagDetection
	 * @return 3d pose of tag.
	 */
	private Transform3d getTagPose(AprilTagDetection tag) {
		// Translation3d https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png
		// Rotation3d Quaternion https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png
		return aprilTagPoseEstimator.estimate(tag);
	}

	/** Outlines and adds helpful data to diplay about a tag in Mat
	 * @param mat the mat to draw the decorations on
	 * @param tag the april tag you want to draw stuff for
	 * @param tagPose3d the 3d pose of that tag
	 */
	private void decorateTagInImage(Mat mat, AprilTagDetection tag, Transform3d tagPose3d) {
		// get distance to tag and convert it to feet

		// final double distanceMM = makePose2d(tagPose3d).getNorm();
		final double distanceMM = tagPose3d.getTranslation().getNorm();

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
		Size tagIdTextSize = addCenteredText(mat, Integer.toString(tag.getId()), 4, 4,
				new Point(tag.getCenterX(), tag.getCenterY()));

		// draw distance to tag under tag Id

		final double distanceFeet = distanceMM / 304.8;
		addCenteredText(mat, String.format("%.1f ft.", distanceFeet), 1, 2,
				new Point(tag.getCenterX(), tag.getCenterY() + (tagIdTextSize.height * 0.7)));
	}

	/** add text centerd to point to mat */
	private Size addCenteredText(Mat mat, String text, int fontScale, int thickness, Point org) {
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

		return textSize;
	}

	/** Check if tag should be consideded by checking if its ID is possible and if its decision margin is high enough  */
	private boolean isTagValid(AprilTagDetection tag) {
		return tag.getId() >= AprilTagValueConstants.MIN_TAG_NUMBER
				&& tag.getId() <= AprilTagValueConstants.MAX_TAG_NUMBER
				&& tag.getDecisionMargin() > AprilTagValueConstants.MIN_APRIL_TAG_DECISION_MARGIN;
	}

	/** 
	 * Get tranform3d location of all detect tags.
	 * 3D pose of tag is Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @return Map with keys as tag id and values as 3D pose of tag. HashMap<tagId, pose3d>.
	*/
	public HashMap<Integer, Transform3d> makeAprilTagLookup(AprilTagDetection[] apirlTags) {
		final HashMap<Integer, Transform3d> tagLocations = new HashMap<>();
		for (AprilTagDetection tag : apirlTags) {
			if (!isTagValid(tag))
				continue;

			final Transform3d pose3d = getTagPose(tag);
			tagLocations.put(tag.getId(), pose3d);

			decorateTagInImage(mat, tag, pose3d);
		}
		return tagLocations;
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
			final AprilTagDetection[] detectedAprilTags = aprilTagDetector.detect(grayMat);

			final HashMap<Integer, Transform3d> tagLocations = makeAprilTagLookup(detectedAprilTags);

			for (int tagId = AprilTagValueConstants.MIN_TAG_NUMBER; tagId <= AprilTagValueConstants.MAX_TAG_NUMBER; tagId++) {
				NetworkTableEntry entry = table.getEntry(String.valueOf(tagId));
				if (tagLocations.containsKey(tagId)) {

					Transform3d pose3d = tagLocations.get(tagId);

					entry.setDoubleArray(new double[] { pose3d.getX(), pose3d.getY(), pose3d.getZ(),
							pose3d.getRotation().getQuaternion().getW(),
							pose3d.getRotation().getX(), pose3d.getRotation().getY(),
							pose3d.getRotation().getZ() });
				} else {
					entry.setDoubleArray(new double[] {});
				}
			}
			outputStream.putFrame(mat);
		}
	}
}
