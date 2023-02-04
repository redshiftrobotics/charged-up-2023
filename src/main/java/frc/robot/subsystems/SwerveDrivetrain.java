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
	private Translation2d velocity;
	private double rotationSpeed;

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

	/** Set all robot movement to either relative to the field forward (true) or relative to the robot forward (false) */
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

	/** Return robot speed as ChassisSpeeds */
	public Translation2d getVelocity() {
		return velocity;
	}

	public double getRotationSpeed() {
		return rotationSpeed;
	}

	/** Set the SwerveModuleState of all modules
	 * 
	 * @param speeds The ChassisSpeeds object to calculate module states
	 * based off forward-backward, left-right, and rotation speeds.
	 */
	public void setSwerveModuleStates(Translation2d velocity, double rotation) {
		this.velocity = velocity;
		this.rotationSpeed = rotation;
		ChassisSpeeds newSpeeds = new ChassisSpeeds(
				this.velocity.getX(), this.velocity.getY(), this.rotationSpeed);

		if (fieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(newSpeeds, gyro.getRotation2d());
		} else {
			speeds = newSpeeds;
		}
		SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(this.speeds);
		moduleFL.setState(moduleStates[0]);
		moduleFR.setState(moduleStates[1]);
		moduleBL.setState(moduleStates[2]);
		moduleBR.setState(moduleStates[3]);
	}

	// calculates odometry and updates the pose
	@Override
	public void periodic() {
		this.pose = odometry.update(gyro.getRotation2d(),
				new SwerveModulePosition[] {
						moduleFL.getPosition(),
						moduleFR.getPosition(),
						moduleBL.getPosition(),
						moduleBR.getPosition() });
	}
}
