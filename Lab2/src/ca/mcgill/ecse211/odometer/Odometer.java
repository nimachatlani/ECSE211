/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * 
 * This class implements the Odometer for Lab2 on the EV3 platform.
 * 
 * @author Nima Chatlani & Valeria Guevara Siguenza 
 * 
 **/

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private double[] position;
  
  //Wheel distances
  private double distL =0;
  private double distR =0;
  
  //Change in theta and displacement
  private double deltaT =0;
  private double deltaD = 0;
  private double dH = 0;
  
  //Change in X and Y
  private double dX = 0;
  private double dY = 0;
  

  //Old and New tacho counts
  private double last_tachoL;
  private double last_tachoR;
  private double nowTachoL;
  private double nowTachoR;
  
  
  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
	
	  //Reset tacho count to prevent infinite fluctuation
	  leftMotor.resetTachoCount(); 
	  rightMotor.resetTachoCount();

	  last_tachoL = leftMotor.getTachoCount();
	  last_tachoR = rightMotor.getTachoCount();

	  long updateStart, updateEnd;

	  while (true) {
		  updateStart = System.currentTimeMillis();

		  leftMotorTachoCount = 0;
		  rightMotorTachoCount =0;

		  //Assign variables to current tacho count
		  nowTachoL = leftMotor.getTachoCount(); 
		  nowTachoR = rightMotor.getTachoCount();

		  // TODO Calculate new robot position based on tachometer counts

		  distL = Math.PI * this.WHEEL_RAD*(nowTachoL - last_tachoL)/180; //compute wheel
		  distR = Math.PI * this.WHEEL_RAD*(nowTachoR - last_tachoR)/180; //compute wheel

		  //Assign previous tacho count to current tacho count
		  last_tachoL = nowTachoL;
		  last_tachoR = nowTachoR;

		  deltaD = (distL-distR); //compute vehicle displacement
		  
		//compute change in heading
		  deltaT = ((deltaD/TRACK)*180)/Math.PI; 
		  dH = (distR+distL)/2;

		  //Compute change in X & Y
		  dX = dH * Math.sin(deltaT);
		  dY = dH * Math.cos(deltaT);



		  // TODO Update odometer values with new calculated values
		  odo.update(dX, dY, deltaT);

		  // this ensures that the odometer only runs once every period
		  updateEnd = System.currentTimeMillis();
		  if (updateEnd - updateStart < ODOMETER_PERIOD) {
			  try {
				  Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
			  } catch (InterruptedException e) {
				  // there is nothing to be done
			  }
		  }
    }
  }

}
