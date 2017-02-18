package org.usfirst.frc.team6063.robot.Events;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class EventBus extends Thread {

	private List<Event> registeredEvents;
	private List<Object> listeners;
	private final BlockingQueue<Event> eventQueue;
	
	public EventBus() {
		registeredEvents = new ArrayList<Event>();
		listeners = new ArrayList<Object>();
		
		eventQueue = new ArrayBlockingQueue<Event>(10);
		Thread consumer1 = new EventConsumerThread(eventQueue);
		Thread consumer2 = new EventConsumerThread(eventQueue);
		
		consumer1.start();
		consumer2.start();
		
		this.start();
	}
	
	public void addListener(Object l) {
		listeners.add(l);
		for (Event e : getRegisteredEvents()) {
			e.addListener(l);
		}
	}
	
	public void registerEvent(Event e) {
		registeredEvents.add(e);
		for (Object l : listeners) {
			e.addListener(l);
		}
	}
	
	synchronized private List<Event> getRegisteredEvents() {
		return registeredEvents;
	}
	
	@Override
	public void run() {
		while (true) {
			for (Event e : getRegisteredEvents()) {
				while (!eventQueue.offer(e));
			}
		}
	}
	
}
