package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

/* The robot drivetrain using Swerve Drive */
public class SwerveDrivetrain extends SubsystemBase {
	// Locations of wheels relative to robot center
	private static final Translation2d locationFL = new Translation2d(
			-SwerveDriveConstants.MODULE_LOCATION_X, SwerveDriveConstants.MODULE_LOCATION_Y);
	private static final Translation2d locationFR = new Translation2d(
			SwerveDriveConstants.MODULE_LOCATION_X, SwerveDriveConstants.MODULE_LOCATION_Y);
	private static final Translation2d locationBL = new Translation2d(
			-SwerveDriveConstants.MODULE_LOCATION_X, -SwerveDriveConstants.MODULE_LOCATION_Y);
	private static final Translation2d locationBR = new Translation2d(
			SwerveDriveConstants.MODULE_LOCATION_X, -SwerveDriveConstants.MODULE_LOCATION_Y);

	private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			locationFL, locationFR, locationBL, locationBR);
	private final SwerveDriveOdometry odometry;
	private ChassisSpeeds speeds = new ChassisSpeeds();
	private boolean fieldRelative = false;

	private final SwerveModule moduleFL;
	private final SwerveModule moduleFR;
	private final SwerveModule moduleBL;
	private final SwerveModule moduleBR;

	private final AHRS gyro; // the gyroscope of the robot - using naxX2 with SPI protocol
	private Pose2d pose = new Pose2d(); // the position of the robot

	/** Creates the SwerveDrivetrain and initializes odometry
	 * 
	 * @param gyro The Gyroscope of the robot. Part of AHRS class corresponding the our gyro.
	 */
	public SwerveDrivetrain(AHRS gyro, SwerveModule swerveModuleFL, SwerveModule swerveModuleFR,
			SwerveModule swerveModuleBL, SwerveModule swerveModuleBR) {
		this.gyro = gyro;
		moduleFL = swerveModuleFL;
		moduleFR = swerveModuleFR;
		moduleBL = swerveModuleBL;
		moduleBR = swerveModuleBR;
		odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),
				new SwerveModulePosition[] {
						moduleFL.getPosition(),
						moduleFR.getPosition(),
						moduleBL.getPosition(),
						moduleBR.getPosition() });
	}

	/** Set the robot's movement to either field or robot relative
	 * @param fieldRelative If true, movement will be field relative
	 */
	public void setFieldRelative(boolean fieldRelative) {
		this.fieldRelative = fieldRelative;
	}

	/** Toggle the robot movement between relative to the field forward and relative to the robot forward */
	public void toggleFieldRelative() {
		fieldRelative = !fieldRelative;
	}

	/** Return robot position as Pose2d */
	public Pose2d getRobotPosition() {
		return pose;
	}

	public double getRobotPitchRotation() {
		return Math.toRadians(gyro.getPitch());
	}

	/** Return robot rotation speed in radians per second. */
	public double getRotationSpeed() {
		return Math.toRadians(gyro.getRate());
	}

	/** Return robot speed as Translation2d
	 * Will return speed depending on fieldRelative.
	 * @return Use getX() and getY() to get the speeds in meters per second. 
	*/
	public Translation2d getVelocity() {
		// Add each module speed to get average robot speed.
		Translation2d robotVelocity = new Translation2d();
		robotVelocity.plus(new Translation2d(moduleFL.getVelocity() / 4, moduleFL.getYawRotation()));
		robotVelocity.plus(new Translation2d(moduleFR.getVelocity() / 4, moduleFR.getYawRotation()));
		robotVelocity.plus(new Translation2d(moduleBL.getVelocity() / 4, moduleBL.getYawRotation()));
		robotVelocity.plus(new Translation2d(moduleBR.getVelocity() / 4, moduleBR.getYawRotation()));

		// Returns speed if fieldRelative is false, otherwise rotates it into fieldRelative and returns.
		if (fieldRelative) {
			return robotVelocity.rotateBy(gyro.getRotation2d().times(-1));
		} else {
			return robotVelocity;
		}
	}

	/** Set the SwerveModuleState of all modules
	 * 
	 * @param speeds The ChassisSpeeds object to calculate module states
	 * 		based off forward-backward, left-right, and rotation speeds.
	 */
	public void setSwerveModuleStates(ChassisSpeeds speeds) {
		if (fieldRelative) {
			this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
		} else {
			this.speeds = speeds;
		}
		SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(this.speeds);
		moduleFL.setState(moduleStates[0]);
		moduleFR.setState(moduleStates[1]);
		moduleBL.setState(moduleStates[2]);
		moduleBR.setState(moduleStates[3]);
	}

	@Override
	public void periodic() {
		// calculates odometry and updates the pose
		this.pose = odometry.update(gyro.getRotation2d(),
				new SwerveModulePosition[] {
						moduleFL.getPosition(),
						moduleFR.getPosition(),
						moduleBL.getPosition(),
						moduleBR.getPosition() });
		SmartDashboard.putNumber("Gyro reading Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro reading Pitch", gyro.getPitch());
		SmartDashboard.putNumber("Gyro reading Yaw", gyro.getYaw());
		SmartDashboard.putNumber("Gyro reading Roll", gyro.getRoll());
		SmartDashboard.putBoolean("Field Relative?", fieldRelative);
		SmartDashboard.putNumber("SpeedsX", speeds.vxMetersPerSecond);
		SmartDashboard.putNumber("SpeedsY", speeds.vyMetersPerSecond);
		SmartDashboard.putNumber("SpeedsR", speeds.omegaRadiansPerSecond);
	}

	/** stops all swerve modules */
	public void stop() {
		moduleFL.stop();
		moduleFR.stop();
		moduleBL.stop();
		moduleBR.stop();
	}
}
