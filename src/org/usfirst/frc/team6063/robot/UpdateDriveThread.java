package org.usfirst.frc.team6063.robot;

public class UpdateDriveThread extends Thread {
	
	private static final double PI = Math.PI;
	
	//Drive speed = (1 + angle * turn_kp) * drive_kp + minSpeed
	private static final double DRIVING_TURNING_KP = 0.5;
	private static final double DRIVING_KP = 0.5;
	
	//Turn speed = kP * angle remaining + min turn speed
	private static final double STATIONARY_TURNING_KP = 0.15;
	private static final double MINIMUM_TURN_SPEED = 0.02;
	
	//Time to wait once position has been reached to ensure it does not overshoot
	private static final long GOOD_POSITION_WAIT_TIME = (long) 250e6; 
	
	
	/*
	 * isDriving       		| True when robot is currently self driving
	 * 							
	 * posIsGood     		| True when robot is at target position,
	 *						| 	it allows the delay to be checked
	 *
	 * isSelfDriveActive	| True when self driving mode is enabled
	 * 						| 	robot will not self drive if this is not
	 * 						|	enabled
	 */
	private boolean isDriving;
	private boolean posIsGood;
	private boolean isSelfDriveActive;
	
	private PositionTracker posTracker;
	private Jeff jeff;
	
	private double[] targetPosition = new double[3];
	
	private double checkGoodDelay;
	
	/**
	 * This thread controls the autonomous driving of the robot.
	 * @param posTracker
	 * @param jeff
	 */
	public UpdateDriveThread(PositionTracker posTracker, Jeff jeff) {
		this.posTracker = posTracker;
		this.jeff = jeff;
	}
	
	@Override
	public void run() {
		while (!Thread.interrupted()) {
			// Set time to wait before starting next iteration
			long delay = System.nanoTime() + (long) 4e6;

			//Only drive if self drive active
			if (isSelfDriveActive)
				updateDrive();

			// Ensure 4ms has passed before looping
			while (System.nanoTime() < delay) if (Thread.interrupted()) return;
		}
	}
	
	/**
	 * @return <b><i>boolean</b></i> true if robot is currently driving
	 */
	public boolean isDriving() {
		return isDriving;
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
	
	/**
	 * Sets position that robot needs to travel to. Does not initiate 
	 * movement however, setIsDriving(true) must be called to start
	 * driving.
	 * @param x position relative to field
	 * @param y position relative to field
	 * @param angle relative to field
	 */
	public void setTargetPosition(double x, double y, double angle) {
		targetPosition = new double[] {x, y, angle};
	}
	
	public void setTargetY(double y) {
		targetPosition[1] = y;
	}
	
	public void setTargetAngle(double angle) {
		targetPosition[2] = angle;
	}
	
	public void setIsDriving(boolean driving) {
		isDriving = driving;
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
	
	//Makes the robot turn on the spot
	private void turnOnSpot(double turnSpeed, double minSpeed) {
		if (turnSpeed < 0) minSpeed = -minSpeed;
		double left = turnSpeed + minSpeed;
		double right = -left;
		System.out.println(left);
		
		jeff.setMotorSpeeds(left, right, true);
	}
	
	//Smallest angle to a
	private double smallestAngle(double a) {
		a = a % (2 * PI);
		if (a < 0) return (a >= -PI) ? a : (a + 2 * PI);
		else return (a <= PI) ? a : (a - 2 * PI);
	}
	
	/**
	 * Get angle to x, y using trigonometry rules
	 * 
	 * @param x position
	 * @param y position
	 * @return angle to position x, y.
	 */
	private double getAngleFromDistance(double x, double y) {
		double angle;
		if (y < 0) {
			angle = PI + Math.atan(x / y);
		} else if (x < 0) {
			angle = 2 * PI + Math.atan(x / y);
		} else {
			angle = Math.atan(x / y);
		}
		
		return angle;
	}
	
	public boolean isAtCoordinates() {
		return inRange(posTracker.getXPos(), getTargetX(), 0.05)
				&& inRange(posTracker.getYPos(), getTargetY(), 0.05);
	}
	
	public boolean isAtAngle(double turnRemaining) {
		return Math.abs(turnRemaining) < 0.02;
	}
	
	private double square(double number) {
		return number * number;
	}
	
	private void updateDrive() {
		
		// See if robot needs to be driving
		// isDriving will be set true when a drive command is run
		if (!isDriving)
			return;
		
		double heading = posTracker.getAngle();
		double leftSpeed, rightSpeed;

		// Check to see if robot is where it is supposed to be.
		if (isAtCoordinates()) {

			//Check if angle is correct. If so end, if not turn on the spot.
			double turnRemaining = smallestAngle(getTargetAngle() - (heading % (2 * PI)));
			if (isAtAngle(turnRemaining)) {
				
				jeff.setMotorSpeeds(0, 0, true);
				
				if (posIsGood) {
					
					if (System.nanoTime() > checkGoodDelay) isDriving = false;
				
				} else {
					
					posIsGood = true;
					checkGoodDelay = System.nanoTime() + GOOD_POSITION_WAIT_TIME;
					
				}
				
				return;
				
			} else {
				turnOnSpot(turnRemaining * STATIONARY_TURNING_KP, MINIMUM_TURN_SPEED);
				return;
			}
		} else {
			
			posIsGood = false;

			// Find distance from robot to target position
			double distX = getTargetX() - posTracker.getXPos();
			double distY = getTargetY() - posTracker.getYPos();

			// Transform distance to position relative to robot
			double xPosRel = distX * Math.cos(heading) - distY * Math.sin(heading);
			double yPosRel = distX * Math.sin(heading) + distY * Math.cos(heading);

			//Find linear distance
			double linearDistance = Math.sqrt(square(xPosRel) + square(yPosRel));
			
			//Determine angle to endpoint (Note: not the same as desired angle once at endpoint)
			//From 0 to 2PI
			double angleRel = getAngleFromDistance(xPosRel, yPosRel);

			
			//Check which direction requires less turning
			if (angleRel <= PI / 2 && angleRel >= 2 * PI / 3) {
				
				//Smallest angle to end point relative to robot (from -PI / 2 to PI / 2)
				double smallestAngleRel = smallestAngle(angleRel);
				//If distance to turn is < 0.3 start driving forward, else turn on spot
				if (Math.abs(smallestAngleRel) > 0.3) {
					turnOnSpot(smallestAngleRel * STATIONARY_TURNING_KP, MINIMUM_TURN_SPEED);
					return;
				} else {
					//Feedback loop here only uses proportional factor, can also include I and D for faster 
					//and smoother response (requires more tuning than P loop alone)
					leftSpeed = linearDistance * (1 + smallestAngleRel * DRIVING_TURNING_KP) * DRIVING_KP + MINIMUM_TURN_SPEED;
					rightSpeed = linearDistance * (1 - smallestAngleRel * DRIVING_TURNING_KP) * DRIVING_KP + MINIMUM_TURN_SPEED;
				}
			} else {
				//Get smallest angle robot has to turn if it is to travel backwards
				double smallestAngleRel = smallestAngle(angleRel - PI);
				
				//If distance to turn is < 0.3 start driving forward, else turn on spot
				if (Math.abs(smallestAngleRel) > 0.3) {
					System.out.println("Smallest angle: " + smallestAngleRel);
					turnOnSpot(smallestAngleRel * STATIONARY_TURNING_KP, MINIMUM_TURN_SPEED);
					return;
				} else {
					leftSpeed = linearDistance * (-1 + smallestAngleRel * DRIVING_TURNING_KP) * DRIVING_KP - MINIMUM_TURN_SPEED;
					rightSpeed = linearDistance * (-1 - smallestAngleRel * DRIVING_TURNING_KP) * DRIVING_KP - MINIMUM_TURN_SPEED;
				}
			}
			
			jeff.setMotorSpeeds(leftSpeed, rightSpeed, true);
		}
	}
	
	public void startSelfDrive() {
		isSelfDriveActive = true;
	}
	
	public void stopSelfDrive() {
		isSelfDriveActive = false;
	}
}
