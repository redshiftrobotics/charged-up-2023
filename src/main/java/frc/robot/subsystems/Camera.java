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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
	private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
	private final NetworkTable table;

	protected final HashMap<Integer, Transform3d> seenAprilTagPoses = new HashMap<>();

	public Camera(int cameraID) {
		inst.startClient4("localpiclient");
		inst.setServerTeam(8032);
		inst.startDSClient();
		inst.setServer("localhost");

		table = inst.getTable(String.format("camera-%s-tags", cameraID));
	}

	/** 
	 * Get tranform3d location of tag with specified ID. If it is not found it returns null.
	 * 3D pose of tag is Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @return 3D pose of tag with specified id.
	*/
	public Transform3d getDetectedAprilTagById(int tagId) {
		if (seenAprilTagPoses.isEmpty()) {
			throw new Error("Has not recieved any entries from Coprocessor yet");
		}
		if (!seenAprilTagPoses.containsKey(tagId)) {
			throw new Error("Invalid tag key");
		}
		return seenAprilTagPoses.get(tagId);
	}

	/** 
	 * Get tranform3d location of all detect tags.
	 * 3D pose of tag is Transform3d(Translation3d(x: -right to +left, y: -up to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @return Map with keys as tag id and values as 3D pose of tag. HashMap<tagId, pose3d>.
	*/
	public HashMap<Integer, Transform3d> getDetectedAprilTags() {
		return seenAprilTagPoses;
	}

	/** converts position of tag relative to camera.
	 * 2d pose of tag is Translation2d(x: +foward, y: -right to +left)
	 * @param pose3d a 3d pose of a tag
	 * @return 2d pose of a tag (ignoring verticle).
	 */
	public Translation2d makePose2d(Transform3d pose3d) {
		return new Translation2d(pose3d.getZ(), pose3d.getX());
	}

	/** returns distance to pose in mm
	 * @param pose3d a 3d pose of tag
	 * @return distance to tag in mm
	 */
	public double getDistance(Transform3d pose3d) {
		return pose3d.getTranslation().getNorm();
	}

	/**  distance to pose in mm without verticle distance
	 * @param pose2d a 2d pose of tag
	 * @return distance to tag in mm (ignoring verticle distance)
	 */
	public double getDistance(Translation2d pose2d) {
		return pose2d.getNorm();
	}

	public void periodic() {
		for (String key : table.getKeys()) {
			NetworkTableEntry entry = table.getEntry(key);
			final double[] values = entry.getDoubleArray(new double[] {});

			seenAprilTagPoses.put(Integer.valueOf(key),
					values.length == 0
							? new Transform3d(
									new Translation3d(values[0], values[1], values[2]),
									new Rotation3d(new Quaternion(values[3], values[4], values[5], values[6])))
							: null);
		}

	}
}