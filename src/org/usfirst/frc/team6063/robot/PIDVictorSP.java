package org.usfirst.frc.team6063.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;

public class PIDVictorSP {
	
	VictorSP[] motors;
	
	double targetPower;
	double targetAngularVel;
	double currentAngularVel;
	double lastAngularVel;
	double lastEncoderValue;
	
	private boolean usePID = true;
	
	Encoder enc;

	//Set default values for variables
	private final double MAX_ANGULAR_VEL = 5; //In revolutions per second
	private double kP = 1.2;
	private double kI = 0.02;
	private double kD = 2.5;
	double iDF = 0.95; //Integral dampening factor
	private final double acceleration = 0.5;
	
	int cpr;
	
	public PIDVictorSP(VictorSP[] motors, Encoder encoder, int cyclesPerRevolution) {		
		this.motors = motors;
		this.enc = encoder;
		this.cpr = cyclesPerRevolution;
		
		tPIDLoop.start();
	}
	
	/**
	 * Set values for tuning of PID
	 * <p>
	 * <ul><i>setPIDConstants(<b>double</b> kP, <b>double</b> kI, <b>double</b> kD, <b>double</b> iDF)</i></ul>
	 * <p>
	 * Higher values for constants will cause each 
	 * corresponding value to have a higher effect
	 * on the power sent to the motors.
	 * 
	 * @param kP Constant for effect of error
	 * @param kI Constant of integral
	 * @param kD Constant for effect of derivative
	 * @param iDF Integral dampening factor (0 to 1)
	 */
	public void setPIDConstants(double kP, double kI, double kD, double iDF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.iDF = Math.min(iDF, 1);
	}
	
	
	
	/*
	 * updateWheelSpeeds() is run repetitively on short intervals to get a
	 * specific value for the wheel speeds
	 */
	double integral, lastError;
	private void updateWheelSpeeds (double dT) {
		double newPower;
		
		if (usePID) {
			
			//Calculate desired velocity by adding or subtracting acceleration constant
			if (targetAngularVel < currentAngularVel)
				currentAngularVel = Math.max(currentAngularVel - acceleration * dT, targetAngularVel);
			else
				currentAngularVel = Math.min(currentAngularVel + acceleration * dT, targetAngularVel);
			
			//Calculate angular velocity by dividing change in rotation by deltaT
			double angularVel = ((enc.get() - lastEncoderValue) / cpr) / dT;
			lastEncoderValue = enc.get();
			
			//Calculate error, derivative and integral values
			double error = currentAngularVel - angularVel;
			double derivative = error - lastError;
			integral = (iDF * integral) + error * dT;
			
			//Calculate power estimated to achieve desired velocity
			newPower = motors[0].get() + error * kP * dT + integral * kI + derivative * kD * dT;
			
		} else {
			//If not using PID loop, simply set power to target power
			newPower = targetPower;
		}
		
		//Change motor powers to determined power
		for (int i = 0; i < motors.length; i++) {
			motors[i].set(newPower);
		}
		
	}
	
	/**
	 * Set whether PID should be used
	 * @param usePID
	 */
	public void setUsePID (boolean usePID) {
		this.usePID = usePID;
	}
	
	/**
	 * Set target speed
	 * <p>
	 * <ul><i>setSpeed(<b>double</b> speed)</i></ul>
	 * 
	 * @param speed
	 */
	public void setSpeed(double speed) {
		targetAngularVel = targetPower * MAX_ANGULAR_VEL;
		targetPower = speed;
	}
	
	private Thread tPIDLoop = new Thread() {
		@Override 
		public void run() {
			//Optimize speed by defining variables once
			long lastTime = System.nanoTime();
			double dT;
			long delay;
			lastEncoderValue = enc.get();
			
			while(!Thread.interrupted()) {
				
				//Calculate change in time
				dT = (System.nanoTime() - lastTime) / 1e9;
				lastTime = System.nanoTime();
				delay = System.nanoTime() + (long) 5e7;
				
				//Run updateWheelSpeeds loop
				updateWheelSpeeds(dT);
				
				//Ensure delay has passed until running again
				while(System.nanoTime() < delay);
				
			}
		}
	};
	
}
