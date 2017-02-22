package org.usfirst.frc.team6063.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;

/*
 * TODO: Add code to determine if encoders are working
 * 
 * If encoders aren't working, wheels will be put to full power potentially
 * damaging the robot or interrupting other robots
 */

public class PIDVictorSP {
	
	VictorSP[] motors;
	
	double targetPower;
	double targetAngularVel;
	double targetCurrentAngularVel;
	double currentAngularVel;
	double lastEncoderValue;
	private final static double ACCELERATION = 3;
	
	private boolean debug = false;
	private boolean usePID = true;
	
	Encoder enc;

	//Set default values for variables
	private final double MAX_ANGULAR_VEL = 5; //In revolutions per second
	private double kP = 4.0;
	private double kI = 0.5;
	private double kD = 4.0;
	
	//Integral dampening factor. Higher means integral lasts for less time.
	double iDF = 1;
	
	int cpr;
	
	public PIDVictorSP(VictorSP[] motors, Encoder encoder, int cyclesPerRevolution) {		
		this(motors, encoder, cyclesPerRevolution, false);
	}
	
	public PIDVictorSP(VictorSP[] motors, Encoder encoder, int cyclesPerRevolution, boolean debug) {
		this.motors = motors;
		this.enc = encoder;
		this.cpr = cyclesPerRevolution;
		this.debug = debug;
		
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
	
	public double getAngularVel() {
		return currentAngularVel;
	}
	
	
	
	/*
	 * updateWheelSpeeds() is run repetitively on short intervals to get a
	 * specific value for the wheel speeds
	 */
	double integral, lastError;
	private void updateWheelSpeeds (double dT) {
		double newPower;
		
		//Calculate angular velocity by dividing change in rotation by deltaT
		currentAngularVel = ((enc.get() - lastEncoderValue) / cpr) / dT;
		lastEncoderValue = enc.get();
		
		if (usePID) {			
			//Calculate error, derivative and integral values
			
			if (targetAngularVel - targetCurrentAngularVel > 0) 
				targetCurrentAngularVel = Math.min(
						targetCurrentAngularVel + ACCELERATION * dT, 
						targetAngularVel);
			else targetCurrentAngularVel = Math.max(
					targetCurrentAngularVel - ACCELERATION * dT, 
					targetAngularVel); 
			double error = targetCurrentAngularVel - currentAngularVel;
			double derivative = error - lastError;
			integral = (1 - iDF * dT) * integral + error * dT;
			
			//Calculate power estimated to achieve desired velocity
			newPower = motors[0].get() + dT * (error * kP + integral * kI + derivative * kD);
			
			if (debug) {
				DecimalFormat df = new DecimalFormat("#.##");
				System.out.println(df.format(currentAngularVel) + ", "
						+ df.format(targetAngularVel) + ", "
						+ df.format(error) + ", "
						+ df.format(newPower) + ", "
						+ dT);
			}
			
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
		targetPower = speed;
		targetAngularVel = speed * MAX_ANGULAR_VEL;
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
				delay = System.nanoTime() + (long) 25e6;
				
				//Run updateWheelSpeeds loop
				updateWheelSpeeds(dT);
				
				//Ensure delay has passed until running again
				while(System.nanoTime() < delay);
				
			}
		}
	};
	
}
