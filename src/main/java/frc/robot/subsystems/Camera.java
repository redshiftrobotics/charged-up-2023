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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Camera extends SubsystemBase {
	private final NetworkTableInstance networkTableInst;
	private final NetworkTable networkTable;

	private final HashMap<Integer, DoubleArraySubscriber> aprilTagSubs = new HashMap<>();
	private final HashMap<Integer, Transform3d> aprilTags = new HashMap<>();

	public Camera(int cameraID) {
		// Sets up and connects roboRio to the pi network table
		networkTableInst = NetworkTableInstance.getDefault();
		networkTableInst.setServerTeam(8032);
		networkTableInst.startDSClient();
		networkTableInst.startServer();

		networkTable = networkTableInst.getTable(String.format("camera-%s-tags", cameraID));

		// Sets subscribers for topics
		// TODO test if this works
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

	/** Convert tag location transform with origin at center of robot to a transform with origin at very center front of robot.
	 * @param tranformFromCenter {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at center of robot
	 * @return {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at very front and center of robot
	 */
	public Transform3d adjustTransformToRobotFront(Transform3d tranformFromCenter) {
		return tranformFromCenter.plus(VisionConstants.CENTER_CENTER_TO_FRONT_CENTER);
	}

	/** Convert tag location transform with origin at camera to a transform with origin at very center front of robot.
	 * This method is called to change all {@link edu.wpi.first.math.geometry.Transform3d Transform3d}s to tags as soon as they are gotten from pi coprocessor.
	 * @param tranformFromCamera {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at camera
	 * @return {@link edu.wpi.first.math.geometry.Transform3d Transform3d} with origin at very center of robot
	 */
	private Transform3d adjustTransformToRobotCenter(Transform3d tranformFromCamera) {
		return tranformFromCamera.plus(VisionConstants.CAMERA_POSITION_FROM_CENTER_CENTER);
	}

	/** Get {@link edu.wpi.first.math.geometry.Transform3d Transform3d} location of tag with specified ID. If it is not found it returns null.
	 * 3D pose of tag is <code>Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))</code>
	 * @see <a href="https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png">Translation3d image</a>
	 * @see <a href="https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png">Rotation3d Quaternion image</a>
	 * @return 3D pose of tag with specified id.
	*/
	public Transform3d getDetectedAprilTagById(int tagId) {
		if (!aprilTagSubs.containsKey(tagId)) {
			throw new Error("Invalid tag key");
		}
		return reconstructTransform3d(aprilTagSubs.get(tagId).get());
	}

	/** Get {@link edu.wpi.first.math.geometry.Transform3d Transform3d} location of all found tags. If it is not found it's value will be null.
	 * 3D pose of tag is <code>Transform3d(Translation3d(x: +left to -right, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))</code>
	 * @return HashMap with ag id as key and {@link edu.wpi.first.math.geometry.Transform3d Transform3d} tag location as value
	*/
	public HashMap<Integer, Transform3d> getAllDetectAprilTags() {
		return aprilTags;
		// private final HashMap<Integer, Transform3d> aprilTags = new HashMap<>();
		// for (Integer tagId : aprilTagSubs.keySet()) {
		// 	aprilTags.put(tagId, getDetectedAprilTagById(tagId));
		// }		
	}

	/** Converts position of tag relative to camera.
	 * 2d pose of tag is Transform2d(Translation2d(x: +left to -right, y: +foward), Rotation2d(value: yaw of object))
	 * @param transform a 3d pose of a tag
	 * @return 2d pose of a tag (ignoring verticle).
	 */
	public Transform2d makeTransformInto2d(Transform3d transform) {
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

	/** Distance to pose in mm without verticle distance
	 * @param transform a 2d pose of tag
	 * @return distance to tag in mm (ignoring verticle distance)
	 */
	public double getDistance(Transform2d transform) {
		return transform.getTranslation().getNorm();
	}

	@Override
	public void periodic() {
		// Loops through subscribers to store and output values

		boolean anyTagFound = false;

		for (Integer tagId : aprilTagSubs.keySet()) {
			final DoubleArraySubscriber sub = aprilTagSubs.get(tagId);

			final double[] value = sub.get();
			Transform3d tagLocation = reconstructTransform3d(value);

			aprilTags.put(tagId, tagLocation);

			if (tagLocation != null) {
				// prints time since proggram started, april tag id, and distance to said tag in feet
				System.out.println(String.format("Rio: Found April Tag #%s at distance of %.1f feet [%s]",
						tagId, getDistance(tagLocation) / 304.8, System.currentTimeMillis()));
				anyTagFound = true;
			}
		}

		if (!anyTagFound) {
			System.out.println(String.format("Rio: No Tags [%s]", System.currentTimeMillis()));
		}

	}
}