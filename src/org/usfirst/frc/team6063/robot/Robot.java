package org.usfirst.frc.team6063.robot;

import org.usfirst.frc.team6063.robot.Events.EventBus;
import org.usfirst.frc.team6063.robot.Events.JoystickEvent;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public EventBus eventBus;

	/* Peripherals */
	Joystick joy1; // Joystick
	Jeff jeff;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {		
		jeff = new Jeff();

		joy1 = new Joystick(0);
		
		// Define event bus and register events
		eventBus = new EventBus();
		eventBus.registerEvent(new JoystickEvent(joy1));
		eventBus.addListener(new ButtonListener(jeff));

		System.out.println("robotInit(): Init Complete");
	}

	/*************************************************************************
	 * Automous mode code
	 */

	@Override
	public void autonomousInit() {
		/*
		 * Test autonomous mode should drive to {1, 1} at PI rotation and then back to
		 * {0, 0} at 0 rotation
		 */
		jeff.startSelfDrive();
		
		jeff.drivetoPosition(1, 1, Math.PI);
		
		while(jeff.isBusy() && isAutonomous());
		
		jeff.drivetoPosition(0, 0, 0);
		
		while(jeff.isBusy() && isAutonomous());
	}

	@Override
	public void autonomousPeriodic() {

	}
	
	@Override
	public void disabledInit() {
		jeff.stopSelfDrive();
		jeff.setMotorSpeeds(0, 0, false);
	}

	/*************************************************************************
	 * User operated mode code
	 */

	@Override
	public void teleopInit() {
		jeff.stopSelfDrive();
		jeff.setMotorSpeeds(0, 0, false);
	}

	/**
	 * Periodically adjusts motor speeds based on joystick pressure/power
	 */

	@Override
	public void teleopPeriodic() {
		//Determine desired motor speeds
		double throttle = 0.3 + (0.7 * -(joy1.getThrottle() - 1) / 2);
		double leftSpeed = throttle * (joy1.getX() / 2 - joy1.getY());
		double rightSpeed = throttle * (-joy1.getY() - joy1.getX() / 2);
		
		jeff.setMotorSpeeds(leftSpeed, rightSpeed, false);
		
	}

	@Override
	public void testInit() {
		LiveWindow.setEnabled(false); //Disable LiveWindow, it's annoying
		SmartDashboard.putNumber("kP", 1);
		SmartDashboard.putNumber("kI", 0);
		SmartDashboard.putNumber("kD", 0);
		SmartDashboard.putNumber("iDF", 0);
		SmartDashboard.putNumber("TargetSpeed", 0);
	}

	//Variable to define which mode is currently active for testing.
	//TODO: Maybe make it an option in smartdashboard instead?
	private int testMode = 0;
	
	@Override
	public void testPeriodic() {

		switch (testMode) {
		case 0:
			double kP = SmartDashboard.getNumber("kP", 0);
			double kI = SmartDashboard.getNumber("kI", 0);
			double kD = SmartDashboard.getNumber("kD", 0);
			double iDF = SmartDashboard.getNumber("iDF", 1);
			jeff.setDrivePIDValues(kP, kI, kD, iDF);
			
			double speed = SmartDashboard.getNumber("TargetSpeed", 0);
			jeff.setMotorSpeeds(speed, -speed, true);
			break;

		case 1:
			break;

		default:
			break;
		}
	}

}
