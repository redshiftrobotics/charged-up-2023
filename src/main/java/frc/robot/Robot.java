// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.apriltag.AprilTagDetector.QuadThresholdParameters;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Camera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private RobotContainer robotContainer;

	// private final Camera camModule = new Camera(0);

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();

		Thread visionThread = new Thread(
				() -> {
					UsbCamera camera = CameraServer.startAutomaticCapture();

					int cameraWidth = 640;
					int cameraHeight = 480;

					camera.setResolution(cameraWidth, cameraHeight);

					CvSink cvSink = CameraServer.getVideo();

					Mat mat = new Mat();
					Mat grayMat = new Mat();

					AprilTagDetector aprilTagDetector = new AprilTagDetector();

					Config config = aprilTagDetector.getConfig();
					config.quadSigma = 0.8f; // set Gaussian blue. fix noice with this.

					aprilTagDetector.setConfig(config);

					QuadThresholdParameters quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
					// todo put this stuff in constants file
					quadThreshParams.minClusterPixels = 250;
					quadThreshParams.criticalAngle *= 5; // default is 10
					quadThreshParams.maxLineFitMSE *= 1.5;
					aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

					aprilTagDetector.addFamily("tag16h5");

					while (!Thread.interrupted()) {
						if (cvSink.grabFrame(mat) == 0) {
							continue;
						}

						Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

						AprilTagDetection[] results = aprilTagDetector.detect(grayMat);

						for (AprilTagDetection result : results) {
							System.out.println(result.toString());
						}
					}
					aprilTagDetector.close();
				});
		visionThread.setDaemon(true);
		visionThread.start();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods. This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();

		// for (AprilTagDetection tag : camModule.getDetectedAprilTags()) {
		// 	System.out.println(tag);
		// 	Camera.getDistanceToAprilTag(tag);
		// }
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
