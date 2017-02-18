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
		System.out.println(e.getJoyID());
		
		if (e.getJoyID() == 0) {
			System.out.println(e.getButtonID());
			
			switch (e.getButtonID()) {
			
			case 1:
				jeff.toggleBucket();
				break;
			case 2:
				jeff.toggleInversion();
				break;
				
			}
			
		} else if (e.getJoyID() == 1) {
			
			switch (e.getButtonID()) {
			case 5:
				jeff.setSecondaryJoystickMode(SecondaryJoystickMode.MODE_NET);
				break;
			case 3:
				jeff.setSecondaryJoystickMode(SecondaryJoystickMode.MODE_WINCH);
				break;
			case 1:
				jeff.toggleGate();
				break;
			}
		}

	}

}
