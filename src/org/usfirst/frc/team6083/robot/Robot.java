package org.usfirst.frc.team6083.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
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
	
	private static final int IMG_WIDTH = 640;
	private static final int IMG_HEIGHT = 360;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private RobotDrive drive;
	VictorSP left, right;
	
	
	private final Object imgLock = new Object();
	
	AxisCamera camera = new AxisCamera("Axis Camera 1","axis-camera1.local");
	
	CvSource output;
	
	final double error_range=20;
	final double key = 0.004;
	final double maxspd = 0.4;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		left = new VictorSP(1);
		right = new VictorSP(3);
		
		drive = new RobotDrive(left, right);
		
		output = CameraServer.getInstance().putVideo("Processed: ", 640, 360);
		
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	            }
	        }
	        output.putFrame(pipeline.hsvThresholdOutput());
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
			double speed = 0;
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			double turn = centerX - (IMG_WIDTH / 2);

			if(turn>error_range||turn<-error_range) {
				speed = turn * key;
				if(speed>maxspd) {
					speed = maxspd;
				}
				else if(speed<-maxspd) {
					speed = -maxspd;
				}
			}
			else {
				speed = 0;
			}
			
			
			drive.tankDrive(speed, -speed);
			
			
			SmartDashboard.putNumber("centerX", centerX);
			SmartDashboard.putNumber("turn", turn);
			SmartDashboard.putNumber("speed", speed);
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		double centerX;
		double speed = 0;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		double turn = centerX - (IMG_WIDTH / 2);

		if(turn>error_range||turn<-error_range) {
			speed = turn * key;
			if(speed>maxspd) {
				speed = maxspd;
			}
			else if(speed<-maxspd) {
				speed = -maxspd;
			}
		}
		
		SmartDashboard.putNumber("speed", speed);
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

