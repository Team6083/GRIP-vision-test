package org.usfirst.frc.team6083.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private RobotDrive drive;
	
	private final Object imgLock = new Object();
	
	AxisCamera camera = new AxisCamera("Axis Camera 1","axis-camera1.local");

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		
		
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	            }
	        }
	    });
	    visionThread.start();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			
			double centerX;
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			double turn = centerX - (IMG_WIDTH / 2);
			drive.arcadeDrive(-0.6, turn * 0.005);
			
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		double turn = centerX - (IMG_WIDTH / 2);
		SmartDashboard.putNumber("centerX", centerX);
		SmartDashboard.putNumber("turn", turn);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

