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
	private final NetworkTableInstance inst;
	private final NetworkTable table;

	protected final HashMap<Integer, DoubleArraySubscriber> aprilTagSubs = new HashMap<>();

	public Camera(int cameraID) {

		// Sets up and connects roboRio to the pi network table
		inst = NetworkTableInstance.getDefault();

		inst.startClient4("localpiclient");
		inst.setServerTeam(8032);
		inst.startDSClient();
		inst.setServer("localhost");

		table = inst.getTable(String.format("camera-%s-tags", cameraID));

		// Sets subscribers for topics
		for (int i = AprilTagConstants.MIN_TAG_ID; i <= AprilTagConstants.MAX_TAG_ID; i++) {
			final DoubleArraySubscriber sub = table.getDoubleArrayTopic(Integer.toString(i)).subscribe(new double[] {});
			aprilTagSubs.put(i, sub);
		}
	}

	/** Makes and returns a new transform3d made up of passed in values
	 * @param double[] of values (0: x, 1: y, 2: z, 3-6: quaternion values)
	 * @return new Transform3d of values
	 */
	private Transform3d convertToTransform3d(double[] values) {
		return values.length == 0 ? null
				: new Transform3d(
						new Translation3d(values[0], values[1], values[2]),
						new Rotation3d(new Quaternion(values[3], values[4], values[5], values[6])));
	}

	/** Get tranform3d location of tag with specified ID. If it is not found it returns null.
	 * 3D pose of tag is Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @return 3D pose of tag with specified id.
	*/
	public Transform3d getDetectedAprilTagById(int tagId) {
		if (aprilTagSubs.isEmpty()) {
			throw new Error("Has not recieved any entries from Coprocessor yet");
		}
		if (!aprilTagSubs.containsKey(tagId)) {
			throw new Error("Invalid tag key");
		}
		return convertToTransform3d(aprilTagSubs.get(tagId).get());
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

		// Loops through subscribers and outputs values
		aprilTagSubs.forEach((id, sub) -> {
			final double[] value = sub.get(null);
			System.out.println(String.format("%s: %s", id, value));
		});
		System.out.println();
	}
}