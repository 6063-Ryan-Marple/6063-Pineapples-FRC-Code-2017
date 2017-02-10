package org.usfirst.frc.team6063.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDVictorSP {
	
	VictorSP[] motors;
	double targetAngularVel;
	double currentAngularVel;
	double lastAngularVel;
	double lastEncoderValue;
	
	Encoder enc;

	private final double maxAngularVel = 5; //In revolutions per second
	private double kP = 1.2;
	private double kI = 0.02;
	private double kD = 2.5;
	double iDF = 0.95; //Integral dampening factor
	private final double acceleration = 0.5;
	
	int cpr;
	
	public PIDVictorSP(VictorSP[] motor, Encoder encoder, int cyclesPerRevolution) {
		
		SmartDashboard.putNumber("KP", kP);
		SmartDashboard.putNumber("KI", kI);
		SmartDashboard.putNumber("KD", kD);
		SmartDashboard.putNumber("IDF", iDF);
		
		motors = motor;
		
		enc = encoder;
		
		cpr = cyclesPerRevolution;
		
		tPIDLoop.start();
	}
	
	long lastTime;
	double dT;
	double integral, lastError;
	private void updateWheelSpeeds () {
		kP = SmartDashboard.getNumber("KP", kP);
		kI = SmartDashboard.getNumber("KI", kI);
		kD = SmartDashboard.getNumber("KD", kD);
		iDF = SmartDashboard.getNumber("IDF", iDF);
		
		//Calculate change in time
		dT = (System.nanoTime() - lastTime) / 1e9;
		lastTime = System.nanoTime();
		
		//Calculate velocity
		if (targetAngularVel < currentAngularVel)
			currentAngularVel = Math.max(currentAngularVel - acceleration, targetAngularVel);
		else
			currentAngularVel = Math.min(currentAngularVel + acceleration, targetAngularVel);
		
		double angularVel = ((enc.get() - lastEncoderValue) / cpr) / dT;
		lastEncoderValue = enc.get();
		
		double error = currentAngularVel - angularVel;
		double derivative = error - lastError;
		integral = (iDF * integral) + error * dT;
		double newPower = motors[0].get() + error * kP * dT + integral * kI + derivative * kD * dT;
		
		for (int i = 0; i < motors.length; i++) {
			motors[i].set(newPower);
		}
		
	}
	
	public void setSpeed(double speed) {
		targetAngularVel = speed * maxAngularVel;
	}
	
	private Thread tPIDLoop = new Thread() {
		@Override 
		public void run() {
			lastTime = System.nanoTime();
			lastEncoderValue = enc.get();
			
			while(!Thread.interrupted()) {
				long delay = System.nanoTime() + (long) 5e7;
				updateWheelSpeeds();
				while(System.nanoTime() < delay);
			}
		}
	};
	
}
