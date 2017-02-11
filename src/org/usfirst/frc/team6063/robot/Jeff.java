package org.usfirst.frc.team6063.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.VictorSP;

/*
 * TODO: Add code to determine if encoders are working
 * 
 * Maybe create a function in PIDVictorSP. Have a contingency
 * to potentially attempt to load the gear without encoder feedback.
 * Driving in a straight line shouldn't be too difficult.
 */

public class Jeff {

	private static final double ANGLE_FACTOR = 2;
	private static final int BUCKET_ON_RAW_VAL = 1000;
	private static final int BUCKET_OFF_RAW_VAL = 500;
	private static final double TURNING_CONSTANT = 1;
	private static final double MAX_SPEED = 0.2;
	private static final long GOOD_POSITION_WAIT_TIME = (long) 1e8; //Time to wait once position has been reached

	/**
	 * 
	 * Modes available for secondary joystick
	 * 
	 * @author GLCPineapples
	 */
	public static enum SecondaryJoystickMode {
		MODE_WINCH, MODE_NET
	}

	private PWM actuatorBucket; // Actuator for gear bucket

	/* Position-tracking objects */
	private ADXRS450_Gyro gyro;
	private PositionTracker posTracker;
	private PIDVictorSP mLeftDrive, mRightDrive;
	private VictorSP mNet;
	private Encoder encLeft, encRight; // Encoders

	private double maxNetSpeed = 0.2; // Max motor speed for net

	private double[] targetPosition = new double[3];

	private boolean isBusy = false;

	SecondaryJoystickMode secondaryJoyMode = SecondaryJoystickMode.MODE_NET;

	public Jeff() {

		VictorSP[] leftMotors = new VictorSP[] { new VictorSP(0), new VictorSP(1) };
		VictorSP[] rightMotors = new VictorSP[] { new VictorSP(2), new VictorSP(3) };
		rightMotors[0].setInverted(true);
		rightMotors[1].setInverted(true);

		// Define encoders, reverse direction of B
		encRight = new Encoder(4, 5, false);
		encLeft = new Encoder(0, 1, true);

		mLeftDrive = new PIDVictorSP(leftMotors, encLeft, 360);
		mRightDrive = new PIDVictorSP(rightMotors, encRight, 360);
		mNet = new VictorSP(4);

		// Init the gyroscope
		gyro = new ADXRS450_Gyro();

		// Define linear actuators
		actuatorBucket = new PWM(7);

		// Create new position tracker using encoders A and B
		posTracker = new PositionTracker(gyro, encLeft, encRight, 360, 0.1524, 0.703);
		
		tSelfDriveThread.start();
	}

	public void setSecondaryJoystickMode(SecondaryJoystickMode mode) {
		secondaryJoyMode = mode;
	}

	/**
	 * Sets the speed of the motor which controls the ball net
	 * <p>
	 * <ul>
	 * <i>setNetMotorSpeed(<b>double</b> speed)</i>
	 * </ul>
	 * <p>
	 * The input should be between 0 and 1 which is then scaled by the
	 * maximumNetSpeed which can be changed by <i>setNetMotorSpeed</i>
	 * 
	 * @param speed
	 *            of the net between 0 and 1
	 *            <p>
	 * @see <i>setMaxNetSpeed(<b>double</b> speed)</i>
	 */
	public void setNetMotorSpeed(double speed) {
		// Make sure speed dow not exceed 1 or -1
		speed = Math.max(Math.min(speed, 1), -1);
		mNet.set(speed * maxNetSpeed);
	}

	/**
	 * Set maximum that net motor can travel at.
	 * <p>
	 * <ul>
	 * <i>setMaxNetSpeed(<b>double</b> speed)</i>
	 * </ul>
	 * <p>
	 * When net motor speed is set, the input value is scaled by the max net
	 * speed set here
	 * 
	 * @param speed
	 *            max net motor speed (between 0 and 1)
	 *            <p>
	 * @see <i>setNetMotorSpeed(<b>double</b> speed)</i>
	 */
	public void setMaxNetSpeed(double speed) {
		maxNetSpeed = speed;
	}

	public boolean isBusy() {
		return isBusy;
	}

	public void driveToAngle(double angle) {
		isBusy = true;
		targetPosition[2] = angle;
	}

	public void drivetoPosition(double x, double y, double angle) {
		isBusy = true;
		targetPosition = new double[] {x, y, angle};
	}

	/**
	 * 
	 * @param a
	 *            value to be checked
	 * @param b
	 *            value to check against
	 * @param span
	 *            range that a must be within b
	 * @return
	 */
	private boolean inRange(double a, double b, double span) {
		return Math.abs(a - b) < span;
	}

	/**
	 * @return X value that Jeff is traveling to
	 */
	public double getTargetX() {
		return targetPosition[0];
	}

	/**
	 * @return Y value that Jeff is traveling to
	 */
	public double getTargetY() {
		return targetPosition[1];
	}

	/**
	 * @return Angle that Jeff is moving to
	 */
	public double getTargetAngle() {
		return targetPosition[2];
	}
	
	public void startSelfDrive() {
		selfDriveActive = true;
	}
	
	public void stopSelfDrive() {
		selfDriveActive = false;
	}
	
	//Smallest angle to a
	private double smallestAngle(double a) {
		return a <= Math.PI ? a : 2 * Math.PI - a;
	}

	long checkGoodDelay = 0;
	boolean posIsGood = false;
	
	private void updateDrive() {
		
		// See if robot needs to be doing something
		// isBusy will be set true when a drive command is run
		if (!isBusy)
			return;
		
		double heading = posTracker.getAngle();
		double leftSpeed, rightSpeed;

		// Check to see if robot is where it is supposed to be.
		if (inRange(posTracker.getXPos(), getTargetX(), 0.05)
				&& inRange(posTracker.getYPos(), getTargetY(), 0.05)) {

			//Check if angle is correct. If so end, if not turn on the spot.
			if (Math.abs(smallestAngle(getTargetAngle() - heading)) > 0.02) {
				if (!posIsGood) {
					checkGoodDelay = System.nanoTime() + GOOD_POSITION_WAIT_TIME;
					posIsGood = true;
				} else if (System.nanoTime() > checkGoodDelay) {
					isBusy = false;
				}
				return;
			}
			
			leftSpeed = getTargetAngle() * TURNING_CONSTANT + 0.1;
			rightSpeed = -leftSpeed;

		} else {

			// Find distance from robot to target position
			double distX = getTargetX() - posTracker.getXPos();
			double distY = getTargetY() - posTracker.getYPos();

			// Transform distance to position relative to robot
			double xPosRel = distX * Math.cos(heading) + distY * Math.sin(heading);
			double yPosRel = distX * Math.sin(heading) + distY * Math.cos(heading);

			//Determine angle to endpoint (Note: not the same as desired angle once at endpoint)
			double angleRel = Math.atan(xPosRel / yPosRel);

			// Run at max speed. Robot will turn at full speed
			// if (shortestAngle * ANGLE_FACTOR) > 1
			leftSpeed = 1 + smallestAngle(angleRel) * ANGLE_FACTOR;
			rightSpeed = 1 - smallestAngle(angleRel) * ANGLE_FACTOR;

		}
		
		//Get largest absolute motor speed
		double scale = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

		//Scale down speeds if largest speed > 1
		if (scale > 1) {
			leftSpeed = MAX_SPEED * (leftSpeed / scale);
			rightSpeed = MAX_SPEED * (rightSpeed / scale);
		}
		
		posIsGood = false;
		setMotorSpeeds(leftSpeed, rightSpeed, true);
	}

	/**
	 * Sets speeds of drive base.
	 * <p>
	 * If speed is >1, speeds will be scaled down by largest value <br>
	 * <b>ie.</b> <i>left = 2, right = 1</i> would be scaled to: <i>left = 1,
	 * right = 0.5</i>
	 * 
	 * @param left
	 *            motor value
	 * @param right
	 *            motor value
	 * @param usePID
	 */
	public void setMotorSpeeds(double left, double right, boolean usePID) {
		double scale = Math.max(Math.abs(left), Math.abs(right));

		if (scale > 1) {
			left = left / scale;
			right = right / scale;
		}

		mLeftDrive.setSpeed(left);
		mLeftDrive.setUsePID(usePID);
		mRightDrive.setSpeed(right);
		mRightDrive.setUsePID(usePID);
	}

	private boolean selfDriveActive = false;
	
	// Thread that drives the robot
	private Thread tSelfDriveThread = new Thread() {
		@Override
		public void run() {
			while (!Thread.interrupted()) {
				// Set time to wait before starting next iteration
				long delay = System.nanoTime() + (long) 4e6;

				//Only drive if self drive active
				if (selfDriveActive)
					updateDrive();

				// Ensure 4ms has passed before looping
				while (System.nanoTime() < delay) if (Thread.interrupted()) return;
			}
		}
	};

	/*
	 * Gear collecting bucket code
	 */
	private boolean bucketFlag = false;

	/**
	 * Enable/disable actuator to tilt bucket
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

}
