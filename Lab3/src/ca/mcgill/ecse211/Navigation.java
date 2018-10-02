package ca.mcgill.ecse211;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

import java.util.Map;

import ca.mcgill.ecse211.Odometer;
import ca.mcgill.ecse211.OdometerExceptions;
import lejos.hardware.Sound; 
import ca.mcgill.ecse211.OdometerData;

/**
 * 
 * This class implements the Nacigation class for Lab3 on the EV3 platform.
 * 
 * @author Nima Chatlani & Valeria Guevara Siguenza 
 * 
 **/

//TODO CHANGE VARIABLE NAMES
public class Navigation extends Thread {
	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	/**
	//waypoint array data
	public double[][] WAYPOINTS;
	public double[] POINT1;
	public double[] POINT2;
	public double[] POINT3;
	public int[] POINT4;
	public int[] POINT5;
	**/
	public int A, B, C, D, E, F, G, H, I, J;
	
	
	//speed constants
	public static final int ROTATE_SPEED = 150;
	public static final int FORWARD_SPEED = 250;
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 15.5;
	private static final double TILE_SIZE = 30.48;
	

	
	/**
	//sensor constants
	public float[] lightData;	
	private SampleProvider colorSensor;
	private int count;
	**/
	
	//navigation consant
	public boolean nav;

	/**
	//Sensor objects
	static Port portColor = LocalEV3.get().getPort("S2");
	static SensorModes myColor = new EV3ColorSensor(portColor);
	public static final SampleProvider myColorSample = myColor.getMode("Red");
	static float[] sampleColor = new float[myColor.sampleSize()];
	**/
	
	
	/**
	   * This is the default class constructor. An existing instance of the odometer is used. This is to
	   * ensure thread safety.
	   * 
	   * @throws OdometerExceptions
	   */
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
		//this.colorSensor = myColorSample;
		this.odometer = odometer;
		//this.lightData = new float[colorSensor.sampleSize()]; 
		//this.count = 0;
		this.nav = false;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		

		//Way Points
		this.A = 0;
		this.B = 2;
		
		this.C = 1;
		this.D = 1;
		
		this.E = 2;
		this.F = 2;
		
		this.G = 2;
		this.H = 1;
		
		this.I = 1;
		this.J = 0;
	
		/**

		//Coordinates = Array of Way Points
		this.POINT1[0] = this.A;
		this.POINT1[1] = this.B;
		
		this.POINT2[0] = this.C;
		this.POINT2[1] = this.D;
		
		this.POINT3[0] = this.E;
		this.POINT3[1] = this.F;
		
		this.POINT4[0] = this.G;
		this.POINT4[1] = this.H;
		
		this.POINT5[0] = this.I;
		this.POINT5[1] = this.J;
		
		//Array of Coordinates
		this.WAYPOINTS[0] = this.POINT1;
		this.WAYPOINTS[1] = this.POINT2;
		this.WAYPOINTS[2] = this.POINT3;
		this.WAYPOINTS[3] = this.POINT4;
		this.WAYPOINTS[4] = this.POINT5;
		**/
	}
	
	
	public void travelTo(double x, double y) {
	
		
		 // reset the motors
	    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
	      motor.stop();
	      motor.setAcceleration(3000);
	    }
	    
	    this.nav = true;
	    
		//odometer position data
		double[] odoData = this.odometer.getXYT();
		double odoX = odoData[0];
		double odoY = odoData[1];
	   
	 
	    /**
	    double trajX;
	    double trajY;
	    
	    
	    for (int i = 0; i< 5; i++) {
	    	if (i == 0) {
	    		trajX = x;
	    		trajY = y; 
	    	}
	    	else {
	    		double[] point1 = this.WAYPOINTS[i];
		    	double[] point2;
		    	point2[0]=x;
		    	point2[1]=y;
	    	}
	    	
	    }**/
	    //Calculate trajectory of path and angle
	    double trajX = x - odoX;
	    double trajY = y - odoY;
	    double trajTheta = Math.atan2(trajX,trajY);
	    
	    //if (trajY < 0 && trajTheta < Math.PI) trajTheta += Math.PI; 
		//double distance = Math.sqrt(Math.pow(trajX, 2) + Math.pow(trajY, 2));
	    
	    
	   //rotate to correct angle
	    this.leftMotor.setSpeed(ROTATE_SPEED);
	    this.rightMotor.setSpeed(ROTATE_SPEED);
	    turnTo(trajTheta);
	    
	    double trajLine = Math.hypot(trajX, trajY);
	    
	    //Move forward to correct distance
	    this.leftMotor.setSpeed(FORWARD_SPEED);
	    this.rightMotor.setSpeed(FORWARD_SPEED);
	    this.leftMotor.rotate(convertDistance(trajLine),true);
	    this.rightMotor.rotate(convertDistance(trajLine),false);
	}
	
	public void turnTo(double theta) {
		double[] odoData = this.odometer.getXYT();
		double odoTheta = odoData[2];
		double mainAngle = theta - odoTheta;
		
		//if(mainAngle < 0) mainAngle += 180;
		
		
		if (mainAngle > 0) {
			this.leftMotor.rotate(-convertAngle(mainAngle),true);
			this.rightMotor.rotate(convertAngle(mainAngle),false);
		}
		else if (mainAngle <= 0) {
			this.leftMotor.rotate(convertAngle(mainAngle),true);
			this.rightMotor.rotate(-convertAngle(mainAngle),false);
		}
		
		
		
	/**
		if (mainAngle >= 180) { // turn left
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(mainAngle), true);
			rightMotor.rotate(convertAngle(mainAngle), false);
		}
		else { // turn right
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(mainAngle), true);
			rightMotor.rotate(-convertAngle(mainAngle), false);
		}
		
		
	**/
		
	}
	
	public boolean isNavigating() {
		return this.nav;
	}
	
	/**
	   * 
	   * @throws OdometerExceptions
	   */
	  // run method (required for Thread)
	  public void run() {
	    //long correctionStart, correctionEnd;
	    
	 
	    int light; //sensor light threshold variable
	    double a = TILE_SIZE*this.A;
	    double b = TILE_SIZE*this.B;
	    double c = TILE_SIZE*this.C;
	    double d = TILE_SIZE*this.D;
	    double e = TILE_SIZE*this.E;
	    double f = TILE_SIZE*this.F;
	    double g = TILE_SIZE*this.G;
	    double h = TILE_SIZE*this.H;
	    double i = TILE_SIZE*this.I;
	    double j = TILE_SIZE*this.J;
	    

	    //while (true) {
	      //correctionStart = System.currentTimeMillis();
	      travelTo(a,b);
	      travelTo(c,d);
	      travelTo(e,f);
	      travelTo(g,h);
	      travelTo(i,j);
	    //}
	  }
	  
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance (double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	  }
	  
	  private static int convertAngle(double angle) {
		    return convertDistance(TRACK * angle/2);
		  }

}
