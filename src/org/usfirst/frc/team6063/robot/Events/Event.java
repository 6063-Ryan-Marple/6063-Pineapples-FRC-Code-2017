package org.usfirst.frc.team6063.robot.Events;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public abstract class Event {

	private List<Object> listeners;
	
	public Event() {
		listeners = new ArrayList<Object>();
	}
	
	public void addListener(Object listener) {
		listeners.add(listener);
	}
	
	protected void callEvent() {
		for (Object l : listeners) {
			for (Method m : getListenerMethods(l)) {
				try {
					m.invoke(l, this);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}
	
	private List<Method> getListenerMethods(Object l) {
		List<Method> output = new ArrayList<Method>();
		for (Method m : l.getClass().getMethods()) {
			if (m.isAnnotationPresent(EventHandler.class)) {
				Class<?>[] paramTypes = m.getParameterTypes();
				if (paramTypes.length == 1 && paramTypes[0] == getClass()) {
					output.add(m);
				}
			}
		}
		
		return output;
	}
	
	protected abstract void update();
	
}
