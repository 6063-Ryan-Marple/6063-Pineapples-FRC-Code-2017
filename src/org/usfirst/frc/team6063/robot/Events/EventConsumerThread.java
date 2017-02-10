package org.usfirst.frc.team6063.robot.Events;

import java.util.concurrent.BlockingQueue;

public class EventConsumerThread extends Thread {

	private final BlockingQueue<Event> eventQueue;
	
	public EventConsumerThread(BlockingQueue<Event> q) {
		eventQueue = q;
	}
	
	@Override
	public void run() {
		while (true) {
			try {
				Event e = eventQueue.take();
				e.update();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
	
}
