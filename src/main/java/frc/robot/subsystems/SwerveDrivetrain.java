package frc.robot.subsystems;

import javax.crypto.spec.GCMParameterSpec;
import javax.swing.text.Position;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveModule;

/* The robot drivetrain using Swerve Drive */
public class SwerveDrivetrain extends SubsystemBase {
	// Locations of wheels relative to robot center
	private static final Translation2d locationFL = new Translation2d(
			SwerveDriveConstants.MODULE_LOCATION_X, SwerveDriveConstants.MODULE_LOCATION_Y);
	private static final Translation2d locationFR = new Translation2d(
			SwerveDriveConstants.MODULE_LOCATION_X, -SwerveDriveConstants.MODULE_LOCATION_Y);
	private static final Translation2d locationBL = new Translation2d(
			-SwerveDriveConstants.MODULE_LOCATION_X, SwerveDriveConstants.MODULE_LOCATION_Y);
	private static final Translation2d locationBR = new Translation2d(
			-SwerveDriveConstants.MODULE_LOCATION_X, -SwerveDriveConstants.MODULE_LOCATION_Y);

	private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			locationFL, locationFR, locationBL, locationBR);
	private final SwerveDriveOdometry odometry;
	private ChassisSpeeds speeds;
	private boolean fieldRelative = false;

	// Initialize swerve modules
	private static final SwerveModule moduleFL = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_FL, SwerveDriveConstants.VELOCITY_MOTOR_ID_FL,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_FL);
	private static final SwerveModule moduleFR = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_FR, SwerveDriveConstants.VELOCITY_MOTOR_ID_FR,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_FR);
	private static final SwerveModule moduleBL = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_BL, SwerveDriveConstants.VELOCITY_MOTOR_ID_BL,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_BL);
	private static final SwerveModule moduleBR = new SwerveModule(
			SwerveDriveConstants.ANGULAR_MOTOR_ID_BR, SwerveDriveConstants.VELOCITY_MOTOR_ID_BR,
			SwerveDriveConstants.ANGULAR_MOTOR_ENCODER_ID_BR);

	private final AHRS gyro; // the gyroscope of the robot - using naxX2 with SPI protocol
	private Pose2d pose; // the position of the robot

	/** Creates the SwerveDrivetrain and initializes odometry
	 * 
	 * @param gyro The Gyroscope of the robot. Part of AHRS class corresponding the our gyro.
	 */
	public SwerveDrivetrain(AHRS gyro) {
		this.gyro = gyro;
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

	/** Toggle if the robot will move according to field or robot coordinates */
	public void toggleFieldRelative() {
		fieldRelative = !fieldRelative;
	}

	/** Set the SwerveModuleState of all modules
	 * 
	 * @param speeds The ChassisSpeeds object to calculate module states
	 * based off forward-backward, left-right, and rotation speeds.
	 */
	public void setSwerveModuleStates(ChassisSpeeds speeds) {
		if (fieldRelative) {
			// convert field-relative speeds to robot-relative speeds 
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
	}
}