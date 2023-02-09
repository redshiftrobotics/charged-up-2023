// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.ArrayList;

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

		outputStream = CameraServer.putVideo("RioApriltags", CameraConstants.CAMERA_RESOLUTION_WIDTH,
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
	 * @return Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 */
	public Transform3d estimateTagPose(AprilTagDetection tag) {
		return aprilTagPoseEstimator.estimate(tag);
	}

	/** Returns position of tag relative to camera in 2d.
	 * @param tag a AprilTagDetection
	 * @return simple Translation2d towards april tag
	 */
	public Translation2d estimateTagPose2d(AprilTagDetection tag) {
		final Transform3d pose = estimateTagPose(tag);
		return new Translation2d(pose.getZ(), pose.getX());
	}

	/** Returns distance from a Transform3d using 3d distance formula
	 * @param Transform3d
	 * @return distance from Transform3d
	 */
	public double getDistance3d(Transform3d transform) {
		return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2) + Math.pow(transform.getZ(), 2));
	}

	/** Returns distance from a Translation2d using 2d distance formula
	 * @param Translation2d
	 * @return distance from Translation2d
	 */
	public double getDistance2d(Translation2d translation) {
		return Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// cvSink.grabFrame stores the camera frame in mat. It returns 0 on error.
		final long timeToFrame = cvSink.grabFrame(mat);

		// Creates points to draw from later
		var pt0 = new Point();
		var pt1 = new Point();
		var pt2 = new Point();
		var pt3 = new Point();
		var center = new Point();
		var red = new Scalar(0, 0, 255);
		var blue = new Scalar(70, 7215, 70);

		if (timeToFrame != 0) {
			// Convert mat to gray scale and store it in grayMat
			Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

			// Detects all AprilTags in grayMat and store them
			detectedAprilTags = aprilTagDetector.detect(grayMat);

			// Loops through each april tag detected, once one is
			for (AprilTagDetection tag : detectedAprilTags) {

				// System.out.println(String.format("x: %s inches, y: %s inches", pose.getX() / 25.4, pose.getY() / 25.4));

				// Gets distance from april tag in MM and Inches
				final Translation2d pose = estimateTagPose2d(tag);
				final double distanceMM = getDistance2d(pose);
				final double distanceInch = distanceMM / 25.4;

				// Assigns positions to points created earlier, updated every frame to track april tag
				pt0.x = tag.getCornerX(0);
				pt1.x = tag.getCornerX(1);
				pt2.x = tag.getCornerX(2);
				pt3.x = tag.getCornerX(3);

				pt0.y = tag.getCornerY(0);
				pt1.y = tag.getCornerY(1);
				pt2.y = tag.getCornerY(2);
				pt3.y = tag.getCornerY(3);

				center.x = tag.getCenterX();
				center.y = tag.getCenterY();

				// Draws lines using points around april tag
				Imgproc.line(mat, pt0, pt1, red, 5);
				Imgproc.line(mat, pt1, pt2, red, 5);
				Imgproc.line(mat, pt2, pt3, red, 5);
				Imgproc.line(mat, pt3, pt0, red, 5);

				// Displays tagId and distance from camera in inches on april tag
				Imgproc.putText(mat, String.format(" %s", tag.getId()), center, Imgproc.FONT_HERSHEY_DUPLEX,
						3, blue, 3);

				Imgproc.putText(mat, String.format("%,.2f", distanceInch), pt0, Imgproc.FONT_HERSHEY_DUPLEX,
						1, blue, 1);

				System.out.println();

			}

			// Outputs the newly drawn on frame to the outputStream
			outputStream.putFrame(mat);
		}
	}

}