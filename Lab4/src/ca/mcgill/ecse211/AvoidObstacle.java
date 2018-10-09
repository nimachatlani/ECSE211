package ca.mcgill.ecse211;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class AvoidObstacle extends Thread implements UltrasonicController {
	private Odometer odometer;
	private int distance;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private boolean bool;

	private Object lock;
	private boolean isNavigating;
	private int bandCenter = 12;

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final int FILTER_OUT = 20;
	private int filterControl;

	public AvoidObstacle(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.lock = new Object();
	}

	public void run() {
		try {
			Thread.sleep(1500);
		} catch (Exception e) {
		}
		travelTo(0, 2);
		travelTo(1, 1);
		travelTo(2, 2);
		travelTo(2, 1);
		travelTo(1, 0);
	}

	public void travelTo(double x, double y) { // makes robot travel to absolute field location

		double dFromX, dFromY, theta, resizeX, resizeY;
		isNavigating = true; // set navigating to true
		resizeX = 30.48 * x; // scale the X value from the target tile x value into cm
		resizeY = 30.48 * y; // scale the Y value from the target tile y value into cm

		double[] odometerData = odometer.getXYT();

		dFromX = resizeX - odometerData[0]; // distance from targeted x coordinate to the current x coordinate

		dFromY = resizeY - odometerData[1]; // distance from targeted y coordinate to the current y coordinate

		theta = Math.atan2(dFromX, dFromY); // theta returned in radians

		if (theta < 0) {
			theta = 2 * Math.PI + theta; // we want to keep the radians as positive values for simplicity
		}
		turnTo(theta); // make robot turn to the required angle
		

		while (dFromX > 0.4 || dFromY > 0.4) {
			if (distance < bandCenter) {
				bool = true;
				while (bool) {
					if (distance < bandCenter) {

						leftMotor.setSpeed(ROTATE_SPEED);
						rightMotor.setSpeed(ROTATE_SPEED);

						leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), true);
						rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), false);

						leftMotor.setSpeed(FORWARD_SPEED);
						rightMotor.setSpeed(FORWARD_SPEED);

						leftMotor.rotate(convertDistance(Lab4.WHEEL_RADIUS, 25), true);
						rightMotor.rotate(convertDistance(Lab4.WHEEL_RADIUS, 25), false);

						leftMotor.setSpeed(ROTATE_SPEED);
						rightMotor.setSpeed(ROTATE_SPEED);

						leftMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), true);
						rightMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, 90.0), false);

						leftMotor.setSpeed(FORWARD_SPEED);
						rightMotor.setSpeed(FORWARD_SPEED);

						leftMotor.rotate(convertDistance(Lab4.WHEEL_RADIUS, 15), true);
						rightMotor.rotate(convertDistance(Lab4.WHEEL_RADIUS, 15), false);

						leftMotor.stop(true);
						rightMotor.stop(false);
					}
					isNavigating = false;
					
				}
				travelTo(x, y);
				isNavigating = true;
			}
			dFromX = resizeX - odometerData[0]; // distance from targeted x coordinate to the current x coordinate

			dFromY = resizeY - odometerData[1]; // distance from targeted y coordinate to the current y coordinate

			theta = Math.atan2(dFromX, dFromY); // theta returned in radians

			if (Math.abs(odometerData[2]- theta) >= 0.5) {
				turnTo(theta);
			}
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();

			isNavigating = false;
		}


	}

	public void turnTo(double theta) { // makes robot turn based on the current theta measured by the robot
		isNavigating = true; // set navigation to true

		double[] odometerData = odometer.getXYT(); // get data from odometer
		double delTheta = odometerData[2]; // get the theta value of odometer

		double turningAngle = theta - delTheta; // calculate the absolute turning angle based on the measured theta
												// based on coordinates and the current theta value of the robot
		if (turningAngle < 0) { // again, if the turning angle is negative, we want it to obtain its positive
								// equivalent
			turningAngle = 2 * (Math.PI) - Math.abs(turningAngle);
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if (turningAngle > Math.PI) { // turn left if the turning angle ranges from (181-360) degrees to perform
										// smallest turn
			turningAngle = (2 * Math.PI) - turningAngle;
			leftMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, turningAngle * 57.2958), true);
			rightMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, turningAngle * 57.2958), false);
		} else { // turn right if the turning angle ranges from (0-180) degrees to perform
					// smallest turn
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, turningAngle * 57.2958), true); // convert
																											// turning
																											// angle
																											// into
																											// degrees
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RADIUS, Lab4.TRACK, turningAngle * 57.2958), false); // convert
																											// turning
																											// angle
																											// into
																											// degrees
		}

		isNavigating = false; // set navigation to false
	}

	public boolean isNavigating() { // returns true if another thread has called
									// travelTo() or turnTo() and the method has
									// yet to return, false otherwise
		return isNavigating;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

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

		// TODO: process a movement based on the us distance passed in (P style)
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}