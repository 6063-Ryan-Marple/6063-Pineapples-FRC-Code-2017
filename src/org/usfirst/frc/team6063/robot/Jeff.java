package org.usfirst.frc.team6063.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.VictorSP;

public class Jeff {

	private final double MAX_NET_SPEED = 0.2; //Max motor speed for net
	private final double ANGLE_FACTOR = 1;
	
	/**
	 * 
	 *  Modes available for secondary joystick
	 * 
	 * @author GLCPineapples
	 */
	public static enum SecondaryJoystickMode {
		MODE_WINCH,
		MODE_NET
	}
	
	private PWM actuatorBucket; // Actuator for gear bucket

	/* Position-tracking objects */
	private ADXRS450_Gyro gyro;
	private PositionTracker posTracker;
	private PIDVictorSP mLeftDrive, mRightDrive;
	private VictorSP mNet;
	private Encoder encLeft, encRight; // Encoders
	
	private double[] targetPosition = new double[3];
	
	private boolean isBusy = false;
	
	SecondaryJoystickMode secondaryJoyMode = SecondaryJoystickMode.MODE_NET;
	
	public Jeff() {

		VictorSP[] leftMotors = new VictorSP[] {new VictorSP(0), new VictorSP(1)};
		VictorSP[] rightMotors = new VictorSP[] {new VictorSP(2), new VictorSP(3)};
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
		
		tDriveThread.start();
	}
	
	public void setSecondaryJoystickMode (SecondaryJoystickMode mode) {
		secondaryJoyMode = mode;
	}
	
	public void setNetMotorSpeed(double speed) {
		mNet.set(speed * MAX_NET_SPEED);
	}
	
	public boolean isBusy() {
		return isBusy;
	}
	
	public void driveToAngle(double angle) {
		isBusy = true;
		targetPosition[2] = angle;
	}
	
	
	/**
	 * 
	 * @param a value to be checked
	 * @param b value to check against
	 * @param span range that a must be within b
	 * @return
	 */
	private boolean inRange(double a, double b, double span) {
		return a > (b - span) && a < (b + span);
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
	
	
	
	private void updateDrive() {
		
		//See if robot needs to be doing something
		//isBusy will be set true when a drive command is run
		if (!isBusy) return;
		
		
		//Check to see if robot is where it is supposed to be.
		//If it is, set isBusy to false and end.
		if (inRange(posTracker.getXPos(), getTargetX(), 0.05)
				&& inRange(posTracker.getYPos(), getTargetY(), 0.05)
				&& inRange(posTracker.getAngle(), getTargetAngle(), 0.02)) {
			isBusy = false;
			return;
		}
		
		//Find distance from robot to target position
		double distX = getTargetX() - posTracker.getXPos();
		double distY = getTargetY() - posTracker.getYPos();
		double heading = posTracker.getAngle() % (2 * Math.PI);
		
		//Transform distance to position relative to robot
		double xPosRel = distX * Math.cos(heading) + distY * Math.sin(heading);
		double yPosRel = distX * Math.sin(heading) + distY * Math.cos(heading);
		
		double angleRel = Math.atan(xPosRel / yPosRel);
		
		//Find shortest angle to target
		double shortestAngle = (angleRel < Math.PI) ? angleRel : (angleRel - 2 * Math.PI);
		
		//Run at max speed. Robot will turn at fully speed
		//if (shortestAngle * ANGLE_FACTOR) > 1
		double leftSpeed = 1 + shortestAngle * ANGLE_FACTOR;
		double rightSpeed = 1 - shortestAngle * ANGLE_FACTOR;
		
		double scale = Math.max(leftSpeed, rightSpeed);
		
		if (scale > 1) {
			leftSpeed = leftSpeed / scale;
			rightSpeed = rightSpeed / scale;
		}
		
		mLeftDrive.setSpeed(leftSpeed);
		mRightDrive.setSpeed(rightSpeed);
	}
	
	//Thread that drives the robot
	private Thread tDriveThread = new Thread() {
		@Override
		public void run() {			
			while (!Thread.interrupted()) {
				//Set time to wait before starting next iteration
				long delay = System.nanoTime() + (long) 4e6;
				
				updateDrive();
				
				//Ensure 4ms has passed before looping
				while(System.nanoTime() < delay);
			}
		}
	};
	
	/*************************************************************************
	 * Gear collecting bucket code
	 */
	private boolean bucketFlag = false;
	private static final int BUCKET_ON_RAW_VAL = 1000;
	private static final int BUCKET_OFF_RAW_VAL = 500;

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
