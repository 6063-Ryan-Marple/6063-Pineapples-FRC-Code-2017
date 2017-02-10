package org.usfirst.frc.team6063.robot;

import org.usfirst.frc.team6063.robot.Events.EventHandler;
import org.usfirst.frc.team6063.robot.Events.JoystickEvent;

/* Handles joystick button events */
public class ButtonListener {
	Robot main; /* Main robot object/parent */
	
	/* Constructor */
	public ButtonListener(Robot instance) {
		main = instance;
	}
	
	@EventHandler
	public void onJoystickEvent(JoystickEvent e) {
		switch (e.getButtonID()) {
			/* Button 1: switch gear-collection bucket actuators */
			case 1:
				main.toggleBucket();
				break;
			case 2:
				main.toggleFuelMode();
				break;
			}
		
	}
	
}
