package org.usfirst.frc.team6063.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;

public class PositionTracker {
	
	ADXRS450_Gyro gyro;
	Encoder[] encoders;
	
	public double[] position = new double[3];
	public double[] lastEncPos = new double[2];	
	
	// Distance between wheels divided by 2
	double rotationRadius;
	double gyroOffset;
	int cyclesPerRev;
	int currentRevs;
	double wheelDiameter;
	
	/* Constructor: PositionTracker(gyroObj, leftEncObj, rightEncObj) */
	public PositionTracker(ADXRS450_Gyro gyro, 
			Encoder encLeft, Encoder encRight, int cyclesPerRevolution, 
			double wheelDiameter, double distanceBetweenWheels) {
		
		this.gyro = gyro;
		this.encoders = new Encoder[] {encLeft, encRight};
		
		this.cyclesPerRev = cyclesPerRevolution;
		this.currentRevs = 0;
		
		this.wheelDiameter = wheelDiameter;
		this.rotationRadius = distanceBetweenWheels / 2;
		
		this.lastEncPos[0] = encLeft.get();
		this.lastEncPos[1] = encRight.get();
		
		positionThread.start();
		
		System.out.println("Starting position tracker...");
	}

	/**
	 * Resets position of robot to [0, 0, 0]
	 */
	public void resetPosition() {
		position = new double[3];
		gyroOffset = 0;
	}
	
	/**
	 * Sets current robot position to X, Y and angle defined
	 * 
	 * @param x position of robot in meters
	 * @param y position of robot in meters
	 * @param angle Angle of robot in radians
	 */
	public void setPosition(double x, double y, double angle) {
		position = new double[] {x, y, angle};
		gyroOffset = angle;
		gyro.reset();
	}
	
	/**
	 * Updates position based on encoders.
	 */
	double[] dRot = new double[2];
	double[] dPos = new double[2];
	double dLinearPos = 0;
	private void updatePosition() {
		/* Do for each encoder */
		for (int i = 0; i < encoders.length; i++) {
			dRot[i] = (encoders[i].get() - lastEncPos[i]) / this.cyclesPerRev;
			dPos[i] = dRot[i] * wheelDiameter * Math.PI;
			lastEncPos[i] = encoders[i].get();
		}
		
		dLinearPos = (dPos[0] + dPos[1]) / 2;
		
		position[2] = Math.toRadians(gyro.getAngle()) + gyroOffset;
		position[0] += dLinearPos * Math.sin(position[2]);
		position[1] += dLinearPos * Math.cos(position[2]);
	}
	
	private Thread positionThread = new Thread() {
		@Override
		public void run() {
			while (!Thread.interrupted()) {
				long delay = System.nanoTime() + (long) 5e7;
				updatePosition();
				while(System.nanoTime() < delay);
			}
		}
	};


	/**
	 * Get X position of robot relative to field in metres
	 * @return double X
	 */
	public double getXPos() {
		return this.position[0];
	}
	
	/**
	 * Get Y position of robot relative to field in metres
	 * @return double X
	 */	
	public double getYPos() {
		return this.position[1];
	}
	
	/**
	 * Get angle position of robot relative to field in radians
	 * @return double angle
	 */
	public double getAngle() {
		return this.position[2];
	}
}
