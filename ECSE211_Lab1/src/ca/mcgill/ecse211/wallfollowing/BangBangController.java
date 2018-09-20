package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;
import lejos.hardware.Button; 
import lejos.hardware.ev3.LocalEV3; 
import lejos.hardware.lcd.TextLCD; 
import lejos.hardware.port.Port; 
import lejos.hardware.sensor.EV3TouchSensor; 
import lejos.hardware.sensor.EV3UltrasonicSensor; 
import lejos.hardware.sensor.SensorModes; 
import lejos.robotics.RegulatedMotor; 
import lejos.robotics.SampleProvider; 
import lejos.utility.Timer; 
import lejos.utility.TimerListener;
import java.lang.Math;
import java.lang.Thread;

/**
 * 
 * This class implements the Bang Bang Controller for Lab1 on the EV3 platform.
 * 
 * @author Nima Chatlani & Valeria Guevara Siguenza 
 * 
 **/

public class BangBangController implements UltrasonicController {

//Declaring Constants
  private final int bandCenter; //Ideal distance from wall
  private final int bandwidth; //Distance threshold
  private final int motorLow; //Low Speed
  private final int motorHigh; //High Speed
  private int distance=0; //Actual distance from wall
  private static int distError = 0; //Difference in actual vs ideal distance
  
  
  /**
   * 
   * This method assigns values to the constants
   * 
   * @author Nima Chatlani & Valeria Guevara Siguenza 
   * 
   * @param bandCenter, bandwidth, motorLow, motorHigh
   * 
   **/

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = 25; //Ideal distance is 25cm from the wall
    this.bandwidth = 3; 
    this.motorLow = 80; 
    this.motorHigh = 150;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  /**
   * 
   * This method Implements the Bang Bang Controller
   * 
   * @author Nima Chatlani & Valeria Guevara Siguenza 
   * 
   * @param distance 
   **/
  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    distError = bandCenter - distance; //compute error
    
  //within limits same speed
  //checks whether distance error falls within accepted range
    if (Math.abs(distError) <= bandwidth) { 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); //If distance is within range, keep speed constant(moving forward)
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    //too close to the wall
    //Check that distance error is too large, meaning EV3 is too close to the wall
      else if (distError > 0 && distError <= 20) { 
      	//decrease leftMotor & increase right motor to create motion away from the wall
      	WallFollowingLab.leftMotor.setSpeed(motorLow/2); 
      	WallFollowingLab.rightMotor.setSpeed(motorHigh*2);
      	WallFollowingLab.leftMotor.forward();
      	WallFollowingLab.rightMotor.forward();
      }
    

    
  //Checks whether distance error exceeds 16, this means that the EV3
  //Is too close for a simple turn to move it away from the wall, so
  //We have it go completely backwards to re-adjust
    else if(distError > 20) { 
    	//At constant speed move both motors backwards
    	WallFollowingLab.leftMotor.setSpeed(motorHigh+motorLow);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+motorLow);
    	WallFollowingLab.leftMotor.backward();
    	WallFollowingLab.rightMotor.backward();
    }
    
  //Too far form the wall
  //Checks whether distance Error is below 0 meaning the EV3 has gone too far away from the wall
    else if (distError < 0) { 
    	//Decrease right motor and increase left motor to create motion toward the wall
    	WallFollowingLab.leftMotor.setSpeed(motorHigh+20);
    	WallFollowingLab.rightMotor.setSpeed(motorLow);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    

      
  
  }
 

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
