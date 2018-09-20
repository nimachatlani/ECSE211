package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * 
 * This class implements the P Controller for Lab1 on the EV3 platform.
 * 
 * @author Nima Chatlani & Valeria Guevara Siguenza 
 * 
 **/

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 50;
  
  //Variables
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int distanceError = 0;
 
  /**
   * 
   * This method assigns values to the constants
   * 
   * @author Nima Chatlani & Valeria Guevara Siguenza 
   * 
   * @param bandCenter, bandwidth, filterControl
   * 
   **/

  public PController(int bandCenter, int bandWidth) {	//constructor
    this.bandCenter = 20;
    this.bandWidth = 3;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  
  /**
   * 
   * This method Implements the P Controller
   * 
   * @author Nima Chatlani & Valeria Guevara Siguenza 
   * 
   * @param distance 
   **/
  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    distanceError = bandCenter - distance; //compute error
    int diff = calculateGain(distanceError);

    if (distanceError > 0 && distanceError <= 13 ) { //too close to the wall 
	    	WallFollowingLab.leftMotor.setSpeed(0);
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.forward();
    }
    else if(distanceError > 13) { //go backwards bc way too close
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+FILTER_OUT);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+FILTER_OUT);
    	WallFollowingLab.leftMotor.backward();
    	WallFollowingLab.rightMotor.backward();
    }
    else if(distanceError < 0 ) { //too far from the wall
    	
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+(diff*2)+(FILTER_OUT/2));
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	    	WallFollowingLab.leftMotor.forward();
	    	WallFollowingLab.rightMotor.forward();
	    
    }
   
    else if(Math.abs(distanceError) <= bandWidth) { //right distance
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
	    	
  }
  /**
   * 
   * This method Calculates the gain, which is the proportion by
   * which we increase/decrease the motor speed
   * 
   * @author Nima Chatlani & Valeria Guevara Siguenza 
   * 
   * @param distError 
   **/
  
  public int calculateGain(int distError) {	
	  int calcGain = 0;
	  int propConstant = 3;
	  int maxGain = 75;
	  
	  calcGain = propConstant*Math.abs(distError);	//Compute gain
	  
	  if (calcGain >= maxGain) {	//Calculated gain is too large so we default to the maximum gain 
		  calcGain = maxGain;
	  }
	  
	  return calcGain;
  }


  @Override
  public int readUSDistance() {
    return this.distance;
 
  }

}
