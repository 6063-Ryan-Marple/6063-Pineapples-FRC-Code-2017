package org.usfirst.frc.team6063.robot;

import org.usfirst.frc.team6063.robot.Jeff.SecondaryJoystickMode;
import org.usfirst.frc.team6063.robot.Events.EventHandler;
import org.usfirst.frc.team6063.robot.Events.JoystickEvent;

/* Handles joystick button events */
public class ButtonListener {
	Jeff jeff; /* Main robot object/parent */
	
	/* Constructor */
	public ButtonListener(Jeff instance) {
		jeff = instance;
	}
	
	@EventHandler
	public void onJoystickEvent(JoystickEvent e) {
		switch (e.getButtonID()) {
			case 1:
				jeff.setSecondaryJoystickMode(SecondaryJoystickMode.MODE_NET);
				break;
			case 2:
				jeff.setSecondaryJoystickMode(SecondaryJoystickMode.MODE_WINCH);
				break;
			}
		
	}
	
}
