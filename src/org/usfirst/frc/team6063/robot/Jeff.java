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
	private static final int GATE_UP_RAW_VAL = 2000;
	private static final int GATE_DOWN_RAW_VAL = 1100;
	
	private static final double MAX_NET_SPEED = 0.5; // Max motor speed for net
	private static final double MAX_WINCH_SPEED = 1;
	private static final double MAX_AUTONOMOUS_SPEED = 1;

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
	private PWM actuatorGate;

	/* Position-tracking objects */
	private ADXRS450_Gyro gyro;
	private PositionTracker posTracker;
	private PIDVictorSP mLeftDrive, mRightDrive;
	private VictorSP mNet, mWinch;
	private Encoder encLeft, encRight; // Encoders
	private UpdateDriveThread tUpdateDrive;
	
	private boolean isGateUp;
	private boolean isBucketOut;

	SecondaryJoystickMode secondaryJoyMode = SecondaryJoystickMode.MODE_NET;

	public Jeff() {

		VictorSP[] leftMotors = new VictorSP[] { new VictorSP(0), new VictorSP(1) };
		VictorSP[] rightMotors = new VictorSP[] { new VictorSP(2), new VictorSP(3) };
		
		// Define linear actuators
		actuatorGate = new PWM(6);
		actuatorBucket = new PWM(7);

		// Define encoders, reverse direction of B
		encRight = new Encoder(4, 5, false);
		encLeft = new Encoder(0, 1, true);

		mLeftDrive = new PIDVictorSP(leftMotors, encLeft, 360);
		mRightDrive = new PIDVictorSP(rightMotors, encRight, 360);
		mNet = new VictorSP(4);
		mWinch = new VictorSP(8);

		// Init the gyroscope
		gyro = new ADXRS450_Gyro();

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
	public void setSecondaryMotorSpeed(double speed) {
		// Make sure speed does not exceed 1 or -1
		speed = Math.max(Math.min(speed, 1), -1) ;
		switch (secondaryJoyMode) {
		case MODE_NET:
			mNet.set(speed * MAX_NET_SPEED);
			break;
		case MODE_WINCH:
			mWinch.set(speed * MAX_WINCH_SPEED);
			break;
			
		}
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

	/**
	 * Enable/disable actuator to tilt bucket
	 */
	public void toggleBucket() {
		/* Enable/disable actuator for bucket */
		if (isBucketOut) {
			actuatorBucket.setRaw(BUCKET_OFF_RAW_VAL);
		} else {
			actuatorBucket.setRaw(BUCKET_ON_RAW_VAL);
		}

		/* Toggle flag */
		isBucketOut = !isBucketOut;
	}
	
	public void toggleGate() {
		if (isGateUp) {
			actuatorGate.setRaw(GATE_DOWN_RAW_VAL);
		} else {
			actuatorGate.setRaw(GATE_UP_RAW_VAL);
		}
		
		isGateUp = !isGateUp;
	}

}
