package org.usfirst.frc.team6063.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraProcessingThread extends Thread {
	CameraServer cs;
	CvSource outputStream;
	CvSink cvSink;
	Size res;
	Mat mIn, mScaled, mHSV, mOut;
	private int frameRate;
	
	public CameraProcessingThread(int width, int height, int frameRate) {
		res = new Size(width, height);
		outputStream = CameraServer.getInstance().putVideo("cvCam", width, height);
		cvSink = CameraServer.getInstance().getVideo();
		
		this.frameRate = frameRate;
		
		mIn = new Mat();
		mScaled = new Mat();
		mHSV = new Mat();
		mOut = new Mat();
		
		this.start();
	}
	
	private void update(double dT) {
		cvSink.grabFrame(mIn);
		
		int FPS = Math.round((float) (1 / dT));
		Imgproc.resize(mIn, mScaled, res);
		
		//Add text to the image indicating the FPS
		Scalar white = new Scalar(255, 255, 255);
		Point pointFPS = new Point(10, 10);
		Imgproc.putText(mScaled, "FPS: " + FPS, pointFPS, Core.FONT_HERSHEY_COMPLEX, 1, white, 2, 8, true);
		
		
		//Imgproc.cvtColor(mScaled, mHSV, Imgproc.COLOR_BGR2HSV);
		
		outputStream.putFrame(mScaled);
	}
	
	@Override
	public void run() {
		long lastTime = System.nanoTime();
		while (!Thread.interrupted()) {
			long delay = System.nanoTime() + (int) (1e9 / frameRate);
			double dT = lastTime / System.nanoTime();
			lastTime = System.nanoTime();
			update(dT);
			while (System.nanoTime() < delay);
		}
	}
	
}
