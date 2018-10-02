package ca.mcgill.ecse211;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/*
 * Lab3 Class
 * Implements Odometry.java, OdometryCorrection.java
 *
 */

public class Lab3 {



	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1; //Measured wheel radius
	public static final double TRACK = 15.5; //Measured distance between wheels





	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		


		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		//OdometryCorrection odometryCorrection = new OdometryCorrection(); 
		Display odometryDisplay = new Display(lcd); // No need to change
		//Navigation object
		Navigation nav = new Navigation(odometer,leftMotor,rightMotor);
		


		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("  < Left >   ", 0, 0);
			lcd.drawString("  Navigation ", 0, 1);
	

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
	

		if (buttonChoice == Button.ID_LEFT) {
			
			nav.start();
			// Start odometer and display threads
	      Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      Thread odoDisplayThread = new Thread(odometryDisplay);
	      odoDisplayThread.start();
			
		}

	 else {
			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("Incorrect button pressed", 0, 0);
		}
	

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

	
}





