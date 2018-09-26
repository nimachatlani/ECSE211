/*
 * OdometryCorrection.java
 *
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab2.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound; 

/**
 * 
 * This class implements the OdometerCorrection for Lab2 on the EV3 platform.
 * 
 * @author Nima Chatlani & Valeria Guevara Siguenza 
 * 
 **/

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double OFFSET = 7.5; //Sensor Center
  private Odometer odometer;
  public float[] lightData;
  private static final double TILE_SIZE = 30.48;
  private SampleProvider colorSensor;
  private int count;
  
  
  //Sensor objects
  static Port portColor = LocalEV3.get().getPort("S2");
  static SensorModes myColor = new EV3ColorSensor(portColor);
  public static final SampleProvider myColorSample = myColor.getMode("Red");
  static float[] sampleColor = new float[myColor.sampleSize()];
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {
	this.colorSensor = myColorSample;
    this.odometer = Odometer.getOdometer();
    this.lightData = new float[colorSensor.sampleSize()]; 
    this.count = 0;
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int light; //sensor light threshold variable

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      
      //Detect lines 
      colorSensor.fetchSample(lightData, 0); 
      light = (int)(lightData[0]*1000);
      
      //light threshold for black line, 190 was derived experimentally 
      if (light < 190) {
    	  Sound.beep();
    	  this.count++; //increment black line count
      }
      
      //depending on the number of black lines passed, we set X,Y coordinates 
      //to correct path
      if (count == 0 || count ==3 || count ==9) {
    	  double[] xyt = odometer.getXYT();
    	  double theta = xyt[2];
    	  double x = xyt[0];
    	  double y = xyt[1];
    	  
    	  //Correction based angles 
    	  if((theta>= 45 && theta <= 135) || (theta>= 225 && theta <= 315)) {
    		  double yOffset = Math.sin(theta)*OFFSET;
    		  double newY = y + yOffset;
    		  this.odometer.setY(newY);
    		  
    	  }
    	  else {
    		  double xOffset = Math.cos(theta) * OFFSET;
    		  double newX = x + xOffset;
    		  this.odometer.setX(newX);
    	  }
      }
      else if(count == 1) {
    	  this.odometer.setX(0);
    	  this.odometer.setY(0);
      }
      else if(count == 2) {
    	  this.odometer.setY(TILE_SIZE);
    	  this.odometer.setX(0);
      }

    else if (count == 4) {
    	this.odometer.setY((TILE_SIZE)*2 + (TILE_SIZE)/2);
  	  	this.odometer.setX((TILE_SIZE)/2); 
    }
    else if (count == 5) {
    	this.odometer.setY((TILE_SIZE)*2 + (TILE_SIZE)/2);
  	  	this.odometer.setX(TILE_SIZE); 
    }
     
    else if (count == 7) {
    	this.odometer.setY(TILE_SIZE*2);
  	  	this.odometer.setX(TILE_SIZE*3); 
    }
    else if (count == 8) {
    	this.odometer.setY(TILE_SIZE);
  	  	this.odometer.setX(TILE_SIZE*3); 
    }
    else if (count == 10) {
    	this.odometer.setY(0);
  	  	this.odometer.setX((TILE_SIZE*2) + TILE_SIZE); 
    }
    else if (count == 11) {
    	this.odometer.setY(0);
  	  	this.odometer.setX(TILE_SIZE/2); 
    }
    
      // TODO Calculate new (accurate) robot position


      // TODO Update odometer with new calculated (and more accurate) values

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
