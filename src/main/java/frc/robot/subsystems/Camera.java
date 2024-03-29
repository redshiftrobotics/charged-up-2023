// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
// Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: -backward to +foward), Rotation3d(Quaternion(...)))

public class Camera extends SubsystemBase {

	private final NetworkTableInstance networkTableInst;
	private final NetworkTable networkTable;

	private final HashMap<Integer, DoubleArraySubscriber> aprilTagSubs = new HashMap<>();
	private final HashMap<Integer, Transform3d> aprilTags = new HashMap<>();

	public enum Side {
		FRONT,
		BACK,
		LEFT,
		RIGHT,
	}

	public enum Corner {
		TOP_LEFT,
		TOP_RIGHT,
		BUTTOM_LEFT,
		BUTTOM_RIGHT,
	}

	public Camera(int cameraID) {
		// Sets up and connects roboRio to the pi network table
		networkTableInst = NetworkTableInstance.getDefault();
		networkTableInst.setServerTeam(8032);
		networkTableInst.startDSClient();
		networkTableInst.startServer();

		networkTable = networkTableInst.getTable(String.format("camera-%s-tags", cameraID));

		// Sets subscribers for topics
		for (String topicName : networkTable.getKeys()) {
			final DoubleArrayTopic topic = networkTable.getDoubleArrayTopic(topicName);
			final DoubleArraySubscriber sub = topic.subscribe(new double[] {});

			final int tagId = Integer.getInteger(topicName);
			aprilTagSubs.put(tagId, sub);
			aprilTags.put(tagId, null);
		}
	}

	/** Makes and returns a new {@link edu.wpi.first.math.geometry.Transform3d Transform3d} made up of passed in values
	 * @param values double[] of values [Translation3d.x, Translation3d.y, Translation3d.z, Rotation3d.Quaternion.w, Rotation3d.Quaternion.x, Rotation3d.Quaternion.y, Rotation3d.Quaternion.z]
	 * @return new {@link edu.wpi.first.math.geometry.Transform3d Transform3d} made from values
	 */
	private Transform3d reconstructTransform3d(double[] values) {
		return values.length == 0 ? null
				: new Transform3d(
						new Translation3d(values[0], values[1], values[2]),
						new Rotation3d(new Quaternion(values[3], values[4], values[5], values[6])));
	}

	/** Convert tag location transform with origin at center of robot to a transform with origin at the center of a side of robot
	 * @param tranformFromCenter {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at center center of robot
	 * @return {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at the desired center of a side of robot
	 */
	public Transform3d adjustTransformToSide(Transform3d tranformFromCenter, Side side) {
		int x = 0, y = 0, z = 0;
		switch (side) {
			case FRONT:
				z = VisionConstants.ROBOT_HALF_LENGTH_MM;
				break;
			case BACK:
				z = -VisionConstants.ROBOT_HALF_LENGTH_MM;
				break;
			case LEFT:
				x = VisionConstants.ROBOT_HALF_WIDTH_MM;
				break;
			case RIGHT:
				x = -VisionConstants.ROBOT_HALF_WIDTH_MM;
				break;
			default:
				break;
		}
		return tranformFromCenter.plus(new Transform3d(new Translation3d(x, y, z), new Rotation3d()));
	}

	/** Convert tag location transform with origin at center of robot to a transform with origin at a corner of robot
	 * @param tranformFromCenter {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at center center of robot
	 * @return {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at the desired corner of robot
	 */
	public Transform3d adjustTransformToCorner(Transform3d tranformFromCenter, Corner corner) {
		int x = 0, y = 0, z = 0;
		switch (corner) {
			case TOP_LEFT:
				x = VisionConstants.ROBOT_HALF_WIDTH_MM;
				z = VisionConstants.ROBOT_HALF_LENGTH_MM;
				break;
			case TOP_RIGHT:
				x = -VisionConstants.ROBOT_HALF_WIDTH_MM;
				z = VisionConstants.ROBOT_HALF_LENGTH_MM;
				break;
			case BUTTOM_LEFT:
				x = VisionConstants.ROBOT_HALF_WIDTH_MM;
				z = -VisionConstants.ROBOT_HALF_LENGTH_MM;
				break;
			case BUTTOM_RIGHT:
				x = -VisionConstants.ROBOT_HALF_WIDTH_MM;
				z = -VisionConstants.ROBOT_HALF_LENGTH_MM;
				break;
		}
		return tranformFromCenter.plus(new Transform3d(new Translation3d(x, y, z), new Rotation3d()));
	}

	/** Convert tag location transform with origin at camera to a transform with origin at very center front of robot.
	 * This method is called to change all {@link edu.wpi.first.math.geometry.Transform3d Transform3d}s to tags as soon as they are gotten from pi coprocessor.
	 * @param tranformFromCamera {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at camera
	 * @return {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at very center of robot
	 */
	private Transform3d adjustTransformToRobotCenter(Transform3d tranformFromCamera) {
		return tranformFromCamera.plus(VisionConstants.CAMERA_POSITION_FROM_CENTER_CENTER_MM);
	}

	/** Get {@link edu.wpi.first.math.geometry.Transform3d Transform3d} location of tag with specified ID. If it is not found it returns null.
	 * @return 3D pose of tag with specified id.
	 * @see <a href="https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png">Translation3d image</a>
	 * @see <a href="https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png">Rotation3d Quaternion image</a>
	 */
	public Transform3d getDetectedAprilTagById(int tagId) {
		if (!aprilTags.containsKey(tagId)) {
			throw new Error("Invalid tag key");
		}
		return aprilTags.get(tagId);
	}

	/** Get {@link edu.wpi.first.math.geometry.Transform3d Transform3d} location of all found tags. If it is not found it's value will be null.
	 * @return HashMap with tag id as key and {@link edu.wpi.first.math.geometry.Transform3d Transform3d} tag location as value
	 * @see <a href="https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png">Translation3d image</a>
	 * @see <a href="https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png">Rotation3d Quaternion image</a>
	*/
	public HashMap<Integer, Transform3d> getAllDetectAprilTags() {
		return aprilTags;
	}

	/** Converts position of tag relative to camera.
	 * 2d pose of tag is Transform2d(Translation2d(x: -right to +left, y: -backward to +foward), Rotation2d(value: yaw of object))
	 * @param transform a 3d pose of a tag
	 * @return 2d pose of a tag (ignoring verticle).
	 */
	public Transform2d makeTransform3dInto2d(Transform3d transform) {
		return new Transform2d(new Translation2d(transform.getX(), transform.getZ()),
				new Rotation2d(transform.getRotation().getZ()));
	}

	/** Returns distance to pose in mm
	 * @param transform a 3d pose of tag
	 * @return distance to tag in mm
	 */
	public double getDistance(Transform3d transform) {
		return transform.getTranslation().getNorm();
	}

	@Override
	public void periodic() {
		// Loops through subscribers to store and output values

		for (Integer tagId : aprilTagSubs.keySet()) {
			final DoubleArraySubscriber sub = aprilTagSubs.get(tagId);

			// get deconstructed transform3d values from subscriber. 
			final TimestampedDoubleArray piTagValue = sub.getAtomic();

			// reconstruct transform3d and adjust it so its not centered at camera but instead at very center of robot
			Transform3d tagLocation = adjustTransformToRobotCenter(reconstructTransform3d(piTagValue.value));

			aprilTags.put(tagId, tagLocation);

			if (tagLocation != null) {
				// prints time since proggram started, april tag id, and distance to said tag in feet
				System.out.println(String.format("Found April Tag #%s at distance of %.1f feet [%s]",
						tagId, getDistance(tagLocation) / 304.8, System.currentTimeMillis()));
			}
		}
	}
}