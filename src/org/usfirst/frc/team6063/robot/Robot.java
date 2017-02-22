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
	Joystick joy1, joy2; // Joystick
	Jeff jeff;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {		
		jeff = new Jeff();

		joy1 = new Joystick(0);
		joy2 = new Joystick(1);
		
		// Define event bus and register events
		eventBus = new EventBus();
		eventBus.registerEvent(new JoystickEvent(joy1));
		eventBus.registerEvent(new JoystickEvent(joy2));
		eventBus.addListener(new ButtonListener(jeff));

		new CameraProcessingThread(320, 240, 20);
		
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
		jeff.setPosition(0, 0, 0);
		
		jeff.startSelfDrive();
		
		/*jeff.drivetoPosition(1, 1, Math.PI);
		
		while(jeff.isSelfDriving() && isAutonomous()) updateDashboard();
		
		long delay = System.nanoTime() + (long) 1e9;
		while (System.nanoTime() < delay && isAutonomous()) updateDashboard();
		
		jeff.drivetoPosition(0, 0, 0);*/
		
		while(jeff.isSelfDriving() && isAutonomous()) updateDashboard();
		
		jeff.drivetoPosition(0, 1, Math.PI / 4);
		
		while(jeff.isSelfDriving() && isAutonomous()) updateDashboard();
		
		jeff.drivetoPosition(0, 0, 0);
	}

	@Override
	public void autonomousPeriodic() {

	}
	
	public void updateDashboard() {
		SmartDashboard.putNumber("X", jeff.getX());
		SmartDashboard.putNumber("Y", jeff.getY());
		SmartDashboard.putNumber("Angle", jeff.getAngle());
	}
	
	@Override
	public void disabledInit() {
		jeff.stopSelfDrive();
		jeff.setMotorSpeeds(0, 0, false);
	}
	
	@Override
	public void disabledPeriodic() {
		updateDashboard();
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
		double kP = SmartDashboard.getNumber("kP", 0);
		double kI = SmartDashboard.getNumber("kI", 0);
		double kD = SmartDashboard.getNumber("kD", 0);
		double iDF = SmartDashboard.getNumber("iDF", 1);
		jeff.setDrivePIDValues(kP, kI, kD, iDF);
		updateDashboard();
		double throttle = 0.3 + (0.7 * -(joy1.getThrottle() - 1) / 2);
		
		double forwardSpeed = Math.pow((-joy1.getY()), 2) * throttle;
		if (joy1.getY() > 0) forwardSpeed = -forwardSpeed;
		double leftSpeed = forwardSpeed + (joy1.getX() / 2);
		double rightSpeed = forwardSpeed - (joy1.getX() / 2);
		
		jeff.setSecondaryMotorSpeed(-joy2.getY());
		jeff.setMotorSpeeds(leftSpeed, rightSpeed, false);
		
	}

	@Override
	public void testInit() {
		//PWM actuator = new PWM(6);
		//actuator.setRaw(2000);
		LiveWindow.setEnabled(false); //Disable LiveWindow, it's annoying
		SmartDashboard.putNumber("kP", SmartDashboard.getNumber("kP", 1));
		SmartDashboard.putNumber("kI", SmartDashboard.getNumber("kI", 0));
		SmartDashboard.putNumber("kD", SmartDashboard.getNumber("kD", 0));
		SmartDashboard.putNumber("iDF", SmartDashboard.getNumber("iDF", 0.95));
		SmartDashboard.putNumber("Test Mode", SmartDashboard.getNumber("Test Mode", 0));
		SmartDashboard.putNumber("TargetSpeed", SmartDashboard.getNumber("TargetSpeed", 0));
	}

	//Variable to define which mode is currently active for testing.
	//TODO: Maybe make it an option in smartdashboard instead?
	private int testMode = 0;
	
	@Override
	public void testPeriodic() {
		double speed;
		testMode = (int) SmartDashboard.getNumber("Test Mode", 0);

		switch (testMode) {
		case 0:
			double kP = SmartDashboard.getNumber("kP", 0);
			double kI = SmartDashboard.getNumber("kI", 0);
			double kD = SmartDashboard.getNumber("kD", 0);
			double iDF = SmartDashboard.getNumber("iDF", 1);
			jeff.setDrivePIDValues(kP, kI, kD, iDF);
			
			speed = SmartDashboard.getNumber("TargetSpeed", 0);
			jeff.setMotorSpeeds(-speed, speed, true);
			
			SmartDashboard.putNumber("Left Angular Vel", jeff.getLeftAngularVel());
			SmartDashboard.putNumber("Right Angular Vel", jeff.getRightAngularVel());
			break;

		case 1:
			speed = SmartDashboard.getNumber("TargetSpeed", 0);
			jeff.setMotorSpeeds(-speed, speed, false);
			SmartDashboard.putNumber("Left Angular Vel", jeff.getLeftAngularVel());
			SmartDashboard.putNumber("Right Angular Vel", jeff.getRightAngularVel());
			break;

		default:
			break;
		}
	}

}
