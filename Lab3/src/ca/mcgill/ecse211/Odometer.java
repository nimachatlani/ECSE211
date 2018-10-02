/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
  public final double PI;

  private double[] position;
  
  //private double theta;
  private double x;
  private double y;


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
    //odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = 15.5;
    this.WHEEL_RAD = 2.1;
    this.PI = Math.PI;

    //this.theta = 0;
    this.x = 0;
    this.y= 0;
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
    long updateStart, updateEnd;
    int currentTachoL, currentTachoR, lastTachoL, lastTachoR;
    double distL, distR, dDistance, dTheta, dX, dY;
    
    position = odo.getXYT();
    
    //double[] odometerData = odoData.getXYT();
    //double x = odometerData[0];
    //double y = odometerData[1];
    //double theta = odometerData[2];
    
    
    
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    
    lastTachoL = leftMotor.getTachoCount();
    lastTachoR = rightMotor.getTachoCount();

    while (true) {
      updateStart = System.currentTimeMillis();

      currentTachoL = leftMotor.getTachoCount();
      currentTachoR = rightMotor.getTachoCount();
      
      distL = 2*PI*WHEEL_RAD*(currentTachoL - lastTachoL)/360;
      distR = 2*PI*WHEEL_RAD*(currentTachoR - lastTachoR)/360;
      
      lastTachoL = currentTachoL;
      lastTachoR = currentTachoR;
      
      dDistance = 0.5*(distL+distR);
      dTheta = (distL - distR)/TRACK;
      double theta = Math.toRadians(position[2]);
      
     dX = dDistance*Math.sin(theta);
     dY = dDistance*Math.cos(theta);
     
     theta+= dTheta;
     
     //odoData.setX(x+dX);
     //odoData.setY(y+dY);
     //odoData.setTheta(theta+dTheta);

      // TODO Calculate new robot position based on tachometer counts
      
      // TODO Update odometer values with new calculated values
      odo.update(dX, dY, Math.toDegrees(dTheta));

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
