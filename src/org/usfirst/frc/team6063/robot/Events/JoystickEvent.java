package org.usfirst.frc.team6063.robot.Events;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickEvent extends Event {

	private Joystick joystick;
	private boolean[] lastButtonState;
	private int buttonID;

	public JoystickEvent(Joystick joyInstance) {
		
		joystick = joyInstance;
		lastButtonState = new boolean[joystick.getButtonCount()];
		
	}
	
	public int getButtonID() {
		return buttonID;
	}
	
	public int getJoyID() {
		return joystick.getPort();
	}

	@Override
	public void update() {
		for (int i = 0; i < joystick.getButtonCount(); i++) {
			
			//getRawButton indexes start at 1, not 0.
			if (joystick.getRawButton(i + 1) != lastButtonState[i]) {
				lastButtonState[i] = joystick.getRawButton(i + 1);
				if (joystick.getRawButton(i + 1)) {
					buttonID = i + 1;
					callEvent();
				}
			}
		}

	}

}
