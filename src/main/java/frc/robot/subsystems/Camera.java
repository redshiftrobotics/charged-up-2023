// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;

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
		for (int i = AprilTagConstants.MIN_TAG_ID; i <= AprilTagConstants.MAX_TAG_ID; i++) {
			final DoubleArraySubscriber sub = networkTable.getDoubleArrayTopic(Integer.toString(i))
					.subscribe(new double[] {});
			aprilTagSubs.put(i, sub);

			aprilTags.put(i, null);
		}
	}

	/** Makes and returns a new transform3d made up of passed in values
	 * @param value double[] of values [Translation3d.x, Translation3d.y, Translation3d.z, Rotation3d.Quaternion.w, Rotation3d.Quaternion.x, Rotation3d.Quaternion.y, Rotation3d.Quaternion.z]
	 * @return new Transform3d of values
	 */
	private Transform3d convertToTransform3d(double[] values) {
		return values.length == 0 ? null
				: new Transform3d(
						new Translation3d(values[0], values[1], values[2]),
						new Rotation3d(new Quaternion(values[3], values[4], values[5], values[6])));
	}

	/** Get Tranform3d location of tag with specified ID. If it is not found it returns null.
	 * 3D pose of tag is Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @return 3D pose of tag with specified id.
	*/
	public Transform3d getDetectedAprilTagById(int tagId) {
		if (!aprilTagSubs.containsKey(tagId)) {
			throw new Error("Invalid tag key");
		}
		return convertToTransform3d(aprilTagSubs.get(tagId).get());
	}

	/** Get Tranform3d location of all found tags. If it is not found it's value will be null.
	 * 3D pose of tag is Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @return HashMap with tag id as key and Tranform3d tag location as value
	*/
	public HashMap<Integer, Transform3d> getAllDetectAprilTags() {
		return aprilTags;
		// private final HashMap<Integer, Transform3d> aprilTags = new HashMap<>();
		// for (Integer tagId : aprilTagSubs.keySet()) {
		// 	aprilTags.put(tagId, getDetectedAprilTagById(tagId));
		// }		
	}

	/** Converts position of tag relative to camera.
	 * 2d pose of tag is Translation2d(x: -right to +left, y: +foward)
	 * @param pose3d a 3d pose of a tag
	 * @return 2d pose of a tag (ignoring verticle).
	 */
	public Translation2d makePose2d(Transform3d pose3d) {
		return new Translation2d(pose3d.getX(), pose3d.getZ());
	}

	/** Returns distance to pose in mm
	 * @param pose3d a 3d pose of tag
	 * @return distance to tag in mm
	 */
	public double getDistance(Transform3d pose3d) {
		return pose3d.getTranslation().getNorm();
	}

	/** Distance to pose in mm without verticle distance
	 * @param pose2d a 2d pose of tag
	 * @return distance to tag in mm (ignoring verticle distance)
	 */
	public double getDistance(Translation2d pose2d) {
		return pose2d.getNorm();
	}

	public void periodic() {
		// Loops through subscribers to store and outputs values
		aprilTagSubs.forEach((tagId, sub) -> {
			final double[] value = sub.get();
			final Transform3d tagLocation = convertToTransform3d(value);

			aprilTags.put(tagId, tagLocation);

			if (tagLocation != null) {
				// prints time since proggram started, april tag id, and distance to said tag in feet
				System.out.println(String.format("%s\tFound April Tag #%s at distance of %s feet",
						sub.getAtomic().timestamp, tagId, getDistance(tagLocation) / 304.8));
			}
		});
	}
}