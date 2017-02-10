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
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
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
	PWM actuatorBucket; // Actuator for gear bucket

	/* Position-tracking objects */
	PositionTracker posTracker;
	PIDVictorSP leftDrive, rightDrive;
	Encoder encLeft, encRight; // Encoders

	/* Accelerometer/gyro vars */
	ADXRS450_Gyro gyro;
	double Kp = 0.03; // Proportion coefficient
	int desiredHeading = 0;
	int currAngle; // Current facing angle

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {		
		VictorSP[] leftMotors = new VictorSP[] {new VictorSP(0), new VictorSP(1)};
		VictorSP[] rightMotors = new VictorSP[] {new VictorSP(2), new VictorSP(3)};
		rightMotors[0].setInverted(true);
		rightMotors[1].setInverted(true);
		joy1 = new Joystick(0);

		// Define event bus and register events
		eventBus = new EventBus();
		eventBus.registerEvent(new JoystickEvent(joy1));
		eventBus.addListener(new ButtonListener(this));

		// Define encoders, reverse direction of B
		encRight = new Encoder(4, 5, false);
		encLeft = new Encoder(0, 1, true);

		leftDrive = new PIDVictorSP(leftMotors, encLeft, 360);
		rightDrive = new PIDVictorSP(rightMotors, encRight, 360);
		
		// Init the gyroscope
		gyro = new ADXRS450_Gyro();

		// Define linear actuators
		actuatorBucket = new PWM(7);

		initialiseFuelCatcher();
		
		// Create new position tracker using encoders A and B
		posTracker = new PositionTracker(gyro, encLeft, encRight, 360, 0.1524, 0.703);

		System.out.println("robotInit(): Init Complete");
	}

	/*************************************************************************
	 * Automous mode code
	 */

	@Override
	public void autonomousInit() {
	}

	private static final double speedScaleFactor = 0.05;
	private static final double angleScaleFactor = 0.25;

	@Override
	public void autonomousPeriodic() {
		gyro.reset();
		while (isAutonomous()) {
			double angle = gyro.getAngle(); // get current heading

			double newAngle = angle * Kp * angleScaleFactor;

			/* Definition: robot.arcadeDrive(moveValue, rotateValue) */
		
			// towards
																	// heading 0
			// currAngle += -angle * Kp;
			SmartDashboard.putNumber("Heading", -newAngle);
			Timer.delay(0.004);
		}
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
		posTracker.resetPosition();

		LiveWindow.setEnabled(false);
		
		SmartDashboard.putNumber("SpeedTarget", 0);
		
		// this.timer = new Timer();
	}

	@SuppressWarnings("unused")
	@Override
	public void testPeriodic() {
		double speed = SmartDashboard.getNumber("SpeedTarget", 0);
		leftDrive.setSpeed(speed);
		rightDrive.setSpeed(-speed);
		
		/*
		 * Stub code for timed-testing e.g. drive for 5 seconds, turn left over
		 * 1 second, then drive 5 seconds forward.
		 */
		/*
		 * if (this.timer.get() < 5.0) { robot.drive(0.05, 0); else if
		 * (this.timer.get() < 6.0 { robot.drive(0.00, -1.0
		 * 
		 * ); } else if (this.timer.get() < 10.0){ robot.drive(0.05, 0); }
		 */

		/*
		 * Net-motor control flag enabled - adjust net motor based on joystick
		 * power. Probably map it linearly from some minimum amount of pressure
		 * to the full range of
		 */

		if (fuelMotorFlag) {

			if (false) {
				double joyYPos = joy1.getY();

				// Configurable min/max Ys
				double minYThreshold = 0.05; // the motor only starts if
												// joystick above this magnitude
												// of displacement
				double maxYThreshold = 1.0; // maximum joystick displacement.
											// Above this will simply cap at
											// this.
				double motorSpeed = 0.00;

				double minMotorSpeed = 0.01;
				double maxMotorSpeed = 0.20;
				double gradient = (maxMotorSpeed - minMotorSpeed) / (maxYThreshold - minYThreshold);

				/* Only set motor speed if above threshold for minimum. */
				/*
				 * Joystick pushed backwards - rotate motor in direction such
				 * that net drops back downwards
				 */
				if (joyYPos > minYThreshold) {
					motorSpeed = (gradient * joyYPos) + minMotorSpeed;
				}

				/*
				 * Joystick pushed forward - recalls net to release fuel into
				 * goal
				 */
				if (joyYPos < -minYThreshold) {
					motorSpeed = (gradient * -joyYPos + minMotorSpeed);
				}

				/* Limit motor speeds between -1 and 1 */
				if (motorSpeed > 1.0) {
					motorSpeed = 1.0;
				} else if (motorSpeed < -1.0) {
					motorSpeed = -1.0;
				}
				
				//SmartDashboard.putNumber("Motor Speed", motorSpeed);
				System.out.println("Motor Speed: " + motorSpeed);
	
				fuelMotorObj.set(motorSpeed);
				Timer.delay(0.10);
				return;
			}
			
			
		}

		/*
		switch (testStage) {
		case 0:
			if (posTracker.getAngle() < twoPi) {
				robot.setLeftRightMotorOutputs(0.2, -0.2);
			} else {
				testStage++;
			}
			break;

		case 1:
			if (posTracker.getAngle() > twoPi) {
				robot.setLeftRightMotorOutputs(-0.05, 0.05);
			} else {
				testStage++;
			}
			break;

		default:
			robot.setLeftRightMotorOutputs(0, 0);
			break;
		}
		*/
	}

	/*************************************************************************
	 * Gear collecting bucket code
	 */
	private boolean bucketFlag = false;
	private static final int BUCKET_ON_RAW_VAL = 1000;
	private static final int BUCKET_OFF_RAW_VAL = 500;

	/**
	 * Enable/disable actuator to tilt bucket(?)
	 */
	public void toggleBucket() {
		/* Enable/disable actuator for bucket */
		if (bucketFlag == false) {
			actuatorBucket.setRaw(BUCKET_ON_RAW_VAL);
		} else {
			actuatorBucket.setRaw(BUCKET_OFF_RAW_VAL);
		}

		/* Toggle flag */
		bucketFlag = !bucketFlag;
	}

	/*************************************************************************
	 * Fuel-collecting net code ------------------------ Robot can enter a
	 * "Fuel motor control" state/mode where one of the joysticks can adjust the
	 * motor motor that controls tension on the net.
	 */
	boolean fuelMotorFlag = false;
	private VictorSP fuelMotorObj;

	private void initialiseFuelCatcher() {
		fuelMotorObj = new VictorSP(4); 
		fuelMotorFlag = false;
	}

	public void toggleFuelMode() {
		fuelMotorFlag = !fuelMotorFlag;
	}

	/*************************************************************************
	 * Winch/servomotor code 
	 * --------------------- 
	 * Idea is that pressin joystick button enters "Winch control mode", 
	 * where the left joystick adjusts the motor speeds and the right joystick 
	 * can adjust the servomotor?
	 */
	int servoVal = 0;

	/**
	 * Take the robot up on the rope!
	 */
	public void takeMeUp() {

	}

}
