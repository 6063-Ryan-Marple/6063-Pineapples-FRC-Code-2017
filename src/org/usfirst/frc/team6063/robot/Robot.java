package org.usfirst.frc.team6063.robot;
/* TODO:
 * 1. Winch/lift code
 * 2. Autonomous gear deposit
 * 3. Fix net-motor code (fuel collecting motor
 */

import org.usfirst.frc.team6063.robot.Events.EventBus;
import org.usfirst.frc.team6063.robot.Events.JoystickEvent;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
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
		
	}

	@Override
	public void autonomousPeriodic() {

	}

	/*************************************************************************
	 * User operated mode code
	 */

	@Override
	public void teleopInit() {
		
	}

	/**
	 * Periodically adjusts motor speeds based on joystick pressure/power
	 */

	@Override
	public void teleopPeriodic() {
		double throttle = 0.3 + (0.7 * -(joy1.getThrottle() - 1) / 2);
		double leftSpeed = throttle * (joy1.getX() / 2 - joy1.getY());
		double rightSpeed = throttle * (-joy1.getY() - joy1.getX() / 2);
	}

	/*************************************************************************
	 * Test mode code
	 */
	private int testStage = 0; // Test stage is implemented as a state machine?
	private static final double twoPi = 2 * Math.PI;

	@Override
	public void testInit() {
		LiveWindow.setEnabled(false);
		
		SmartDashboard.putNumber("SpeedTarget", 0);
		
		// this.timer = new Timer();
	}

	//Variable to define which mode is currently active for testing.
	//TODO: Maybe make it an option in smartdashboard instead?
	private int testMode = 0;
	
	@Override
	public void testPeriodic() {
		double speed = SmartDashboard.getNumber("SpeedTarget", 0);

		switch (testMode) {
		case 0:
			break;

		case 1:
			break;

		default:
			break;
		}
	}

}
