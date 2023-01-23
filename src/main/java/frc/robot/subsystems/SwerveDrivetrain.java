package frc.robot.subsystems;

import javax.crypto.spec.GCMParameterSpec;
import javax.swing.text.Position;

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
	
	private final Gyro gyro;
	private Pose2d pose;

	public SwerveDrivetrain(Gyro gyro) {
		this.gyro = gyro;
		odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),
				new SwerveModulePosition[] {
						moduleFL.getPosition(),
						moduleFR.getPosition(),
						moduleBL.getPosition(),
						moduleBR.getPosition() });
	}

	public void setChassisSpeed(ChassisSpeeds speed) {
		SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speed);
		moduleFL.setState(moduleStates[0]);
		moduleFR.setState(moduleStates[1]);
		moduleBL.setState(moduleStates[2]);
		moduleBR.setState(moduleStates[3]);
	}

	@Override
	public void periodic() {
		this.pose = odometry.update(gyro.getRotation2d(),
				new SwerveModulePosition[] { moduleFL.getPosition(),
						moduleFR.getPosition(),
						moduleBL.getPosition(),
						moduleBR.getPosition() });
	}
}
