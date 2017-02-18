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

	private static final int BUCKET_ON_RAW_VAL = 1000;
	private static final int BUCKET_OFF_RAW_VAL = 500;
	
	private static final double MAX_AUTONOMOUS_SPEED = 0.5;

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
	private UpdateDriveThread tUpdateDrive;

	private double maxNetSpeed = 0.2; // Max motor speed for net

	SecondaryJoystickMode secondaryJoyMode = SecondaryJoystickMode.MODE_NET;

	public Jeff() {

		VictorSP[] leftMotors = new VictorSP[] { new VictorSP(0), new VictorSP(1) };
		VictorSP[] rightMotors = new VictorSP[] { new VictorSP(2), new VictorSP(3) };

		actuatorGate = new PWM(6);

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

		tUpdateDrive = new UpdateDriveThread(posTracker, this);
		tUpdateDrive.start();
	}

	public void setSecondaryJoystickMode(SecondaryJoystickMode mode) {
		secondaryJoyMode = mode;
	}

	public void stopSelfDrive() {
		tUpdateDrive.stopSelfDrive();
	}
	
	public void startSelfDrive() {
		tUpdateDrive.startSelfDrive();
	}
	
	public boolean isSelfDriving() {
		return tUpdateDrive.isDriving();
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

	public double getX() {
		return posTracker.getXPos();
	}

	public double getY() {
		return posTracker.getYPos();
	}

	public double getAngle() {
		return posTracker.getAngle();
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

	public void driveToAngle(double angle) {
		tUpdateDrive.setTargetAngle(angle);
		tUpdateDrive.setIsDriving(true);
	}

	public void drivetoPosition(double x, double y, double angle) {
		tUpdateDrive.setTargetPosition(x, y, angle);
		tUpdateDrive.setIsDriving(true);
	}

	public double getLeftAngularVel() {
		return mLeftDrive.getAngularVel();
	}

	public double getRightAngularVel() {
		return mRightDrive.getAngularVel();
	}

	PWM actuatorGate;

	public void toggleGate() {
		if (actuatorGate.getRaw() > 1750) {
			actuatorGate.setRaw(1100);
		} else {
			actuatorGate.setRaw(2000);
		}
	}

	private boolean isInverted = false;

	public void toggleInversion() {
		isInverted = !isInverted;
	}

	public void setPosition(double x, double y, double angle) {
		posTracker.setPosition(x, y, angle);
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
		//Get largest absolute motor speed
		double scale = Math.max(Math.abs(left), Math.abs(right));

		double maxSpeed = 1;
		if (isSelfDriving()) maxSpeed = MAX_AUTONOMOUS_SPEED;
		
		//Scale down speeds if largest speed > 1
		if (scale > maxSpeed) {
			left = MAX_AUTONOMOUS_SPEED * (left / scale);
			right = MAX_AUTONOMOUS_SPEED * (right / scale);
		}

		if (isInverted && !tUpdateDrive.isDriving()) {
			double oldLeft = left;
			left = -right;
			right = -oldLeft;
		}

		mLeftDrive.setSpeed(left);
		mLeftDrive.setUsePID(usePID);
		mRightDrive.setSpeed(right);
		mRightDrive.setUsePID(usePID);
	}

	public void setDrivePIDValues(double kP, double kI, double kD, double iDF) {
		mLeftDrive.setPIDConstants(kP, kI, kD, iDF);
		mRightDrive.setPIDConstants(kP, kI, kD, iDF);
	}

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
			System.out.println("Bucket extending...");
			actuatorBucket.setRaw(BUCKET_ON_RAW_VAL);
		} else {
			System.out.println("Bucket retracting...");
			actuatorBucket.setRaw(BUCKET_OFF_RAW_VAL);
		}

		/* Toggle flag */
		bucketFlag = !bucketFlag;
	}

}
