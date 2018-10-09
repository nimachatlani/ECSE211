// Lab3.java
package ca.mcgill.ecse211;

import lejos.hardware.sensor.*;
import ca.mcgill.ecse211.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
//import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.lcd.TextLCD;

/**
 * We implement an Odometry system that provides a robot's position and
 * orientation, allowing a robot to autonomously navigate a field. Secondly, we
 * implement a correction using a light sensor to improve the Odometer results
 * and return values relative to the defined origin. Lastly, we test and adjust
 * the offset to increase the accuracy of the Odometry system.
 * 
 * @author tianzhufu & lucasbellido
 */

public class Lab4 {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B")); // get
																														// the
																														// leftMotor
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); // get
																														// the
																														// rightMotor
	// public static final Port portColor = LocalEV3.get().getPort("S1"); //get the
	// lightSensor at port S1
	private static final Port usPort = LocalEV3.get().getPort("S1"); // get UltraSonic sensor at port S2
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RADIUS = 2.2;
	public static final double TRACK = 15.5;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// SensorModes myColor = new EV3ColorSensor(portColor);
		// SampleProvider myColorSample = myColor.getMode("Red");
		// float[] sampleColor = new float[myColorSample.sampleSize()];

		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
																	// this instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
																// returned

		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS);
		Navigation navigate = new Navigation(odometer, leftMotor, rightMotor);
		Display odometryDisplay = new Display(lcd, navigate);
		AvoidObstacle avoidObstacle = new AvoidObstacle(odometer, leftMotor, rightMotor);
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, avoidObstacle);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("<   Right for   >", 0, 0);
			lcd.drawString("     navigate    ", 0, 1);
			lcd.drawString("  left for avoid ", 0, 2);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			Thread odoDisplayThread = new Thread(odometryDisplay);
			Thread navigationThread = new Thread(navigate);
			

			if (buttonChoice == Button.ID_RIGHT) {
				navigationThread.start();
				odoThread.start();
				odoDisplayThread.start();

			} else {

			}
			Thread avoidThread = new Thread(avoidObstacle);
			usPoller.start();
			if(!odoThread.isAlive()){
				odoThread.start();
				odoDisplayThread.start();
			}
			avoidThread.start();

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
