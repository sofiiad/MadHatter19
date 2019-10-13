package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.TRACK;
import static ca.mcgill.ecse211.lab3.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.lab3.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.lab3.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.lab3.Resources.leftMotor;
import static ca.mcgill.ecse211.lab3.Resources.rightMotor;
import static ca.mcgill.ecse211.lab3.Resources.leftMotor;
import static ca.mcgill.ecse211.lab3.Resources.rightMotor;
import static ca.mcgill.ecse211.lab3.Resources.ACCELERATION;
import static ca.mcgill.ecse211.lab3.Resources.CM_ERR;
import static ca.mcgill.ecse211.lab3.Resources.DEG_ERR;
import static ca.mcgill.ecse211.lab3.Resources.odometer;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

/**
 * Class that offers static methods used for the robot's navigation.
 */
public class Navigation {

    /**Flag to indicate that the robot is navigation*/
	private static Navigation nav; // Returned as singleton

	/**boolean variable that will be used to indicate if the EV3 is turning or not*/
	public static boolean turning;
	
	/**boolean variable that will be used to indicate if EV3 is still navigating through the course*/
	private static boolean navigating;

	/**Instantiating interrupt variable to false. This indicates that there is no initial obstacle in front of the EV3*/
	private static boolean interrupt = false;

	/**Last x position the robot was at*/
	private static double last_X;
	
	/**Last y position the robot was at*/
	private static double last_Y;

	//Set wheel acceleration
	static {
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * Sets the motor speeds jointly.
	 */
	public static void setSpeeds(float leftSpeed, float rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		if (leftSpeed < 0) {
			leftMotor.backward();
		} else {
			leftMotor.forward();
		}
		if (rightSpeed < 0) {
			rightMotor.backward();
		} else {
			rightMotor.forward();
		}
	}

	/**
	 * Travels to designated position. Used for the Navigation with Obstacle Avoidance setting
	 * 
	 * @param x
	 * @param y
	 * @param avoid
	 */
	public static void travelTo(double x, double y, boolean avoid) {
	    //If the robot has to avoid an obstacle
		if (avoid) {
			ObstacleAvoidance.dest_X = x;
			ObstacleAvoidance.dest_Y = y;

			// This will trigger the state machine running in the obstacleAvoidance thread
			ObstacleAvoidance.traveling = true;
		} 
		//If the robot doesn't need to avoid an obstacle 
		else {
			Navigation.travelTo(x, y);
		}

	}

	/**
	 * Travels to designated position. Used for the Navigation only setting. 
	 * 
	 * @param x
	 * @param y
	 * @param avoid
	 */
	static void travelTo(double x, double y) {

		navigating = true;
		last_X = odometer.getX();
		last_Y = odometer.getY();
		double dX = x - last_X;
		double dY = y - last_Y;

		turnTo(Math.toDegrees(Math.atan2(dX, dY))); // turn to right direction

		travel(dX, dY);

	}
	
	
	/**
	 * Turns to desired position
	 * 
	 * @param x
	 * @param y
	 * */
	static void turnTo(double x, double y) {
		last_X = odometer.getX();
		last_Y = odometer.getY();
		double dX = x - last_X;
		double dY = y - last_Y;

		turnTo(Math.toDegrees(Math.atan2(dX, dY))); // turn to right direction
	}

	/**
	 * The robot travels a certain amount of cm given coordinates
	 * 
	 * @param dX
	 *            absolute displacement in cm
	 * @param dY
	 *            absolute displacement in cm
	 */
	static void travel(double dX, double dY) {
		travel(Math.sqrt(dX * dX + dY * dY));

	}

	/**
	 * Travels a certain amount of cm given distance
	 * 
	 * @param distance
	 */
	static void travel(double distance) {
		double dX, dY;
		last_X = odometer.getX();
		last_Y = odometer.getY();
		travelForward();

		while (!isInterrupt()) {//While no obstacle detected
			dX = Math.abs(odometer.getX() - last_X);
			dY = Math.abs(odometer.getY() - last_Y);
			if (dX * dX + dY * dY >= distance * distance) {// reached destination
				break;
			}
		}

		stopMotors();
		navigating = false;
	}

	/**
	 * Travels forward without stopping
	 */
	static void travelForward() {
		navigating = true;

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		rightMotor.forward();
		leftMotor.forward();

		//navigating = false;
	}

	/**
	 * Turns to specified angle (in degrees) by respecting the minimum turning angle
	 * 
	 * @param theta:
	 *            angles in degrees
	 */
	public static void turnTo(double theta) {

		double odoTheta = odometer.getTheta();

		double rotation = (theta - odoTheta);

		if (rotation > 180) {
			rotation = rotation - 360;

		} else if (rotation < -180) {
			rotation = rotation + 360;

		} else if (Math.abs(rotation) == 180) {
			rotation = Math.abs(rotation);
		}

		turn(rotation);
	}

	/**
	 * Turn clockwise by a certain amount of degrees
	 * 
	 * @param rotation clockwise in degrees
	 *            
	 */
	public static void turn(double rotation) {

		turning = true;

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(rotation), true);
		rightMotor.rotate(-convertAngle(rotation), false);

		turning = false;
	}

	/**
	 * Stops the robot.
	 */
	public static void stopMotors() {
		rightMotor.stop(true);
		leftMotor.stop(false);
		navigating = false;
	}

	/**
	 * Moves robot forward a set distance in cm. Used for navigation with obstacle avoidance setting
	 * 
	 * @param distance
	 * @param avoid
	 */
	public static void goForward(double distance, boolean avoid) {
		double x = odometer.getX() + cos(toRadians(odometer.getTheta())) * distance;
		double y = odometer.getY() + sin(toRadians(odometer.getTheta())) * distance;

		travelTo(x, y, avoid);
	}

	/**
	 * Moves robot forward a set distance in cm. Used for navigation without obstacle avoidance setting.
	 * 
	 * @param distance
	 * 
	 * */
	public static void moveForward(double distance) {
	
		navigating = true;
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(distance), true);
		rightMotor.rotate(convertDistance(distance), false);
		
		navigating = false;
	}
	
	/**
	 * Returns navigation status.
	 * 
	 * @return navigating*/
	public static boolean isNavigating() {
		return navigating;
	}

	/**
	 * Sets navigation status
	 * 
	 * @param navigating
	 * */
	public static void setNavigating(boolean navigating) {
		Navigation.navigating = navigating;
	}

	/**
	 * Converts input distance to the total rotation of each wheel needed to cover
	 * that distance.
	 * 
	 * @param distance
	 * @return the wheel rotations necessary to cover the distance
	 */
	public static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

	/**
	 * Converts input angle to the total rotation of each wheel needed to rotate the
	 * robot by that angle.
	 * 
	 * @param angle
	 * @return the wheel rotations necessary to rotate the robot by the angle
	 */
	public static int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}

	/**
	 * Returns the destination angle.
	 * 
	 * @param x
	 * @param y
	 * @return the destination angle.
	 */
	public static double getDestAngle(double x, double y) {
		
		double minAngle;
		
		last_X = odometer.getX();
		last_Y = odometer.getY();
		double dX = x - last_X;
		double dY = y - last_Y;

		minAngle = Math.toDegrees(Math.atan2(dX, dY)); // turn to right direction		
		
		return minAngle;
	}

	/**
	 * Returns {@code true} when facing destination.
	 * 
	 * @param angle
	 * @return {@code true} when facing destination.
	 */
	public static boolean facingDest(double angle) {
		return abs(angle - odometer.getTheta()) < DEG_ERR;
	}

	/**
	 * Returns the distance in cm between the robot and its next waypoint.
	 * 
	 * @return distance between robot and next waypoint in cm.
	 */
	public static double distanceToNextWayPoint(double dest_X, double dest_Y) {

		double distance;

		double current_X = odometer.getX();
		double current_Y = odometer.getY();

		double dX = current_X - dest_X;
		double dY = current_Y - dest_Y;

		distance = Math.sqrt(Math.pow(dX, 2) - Math.pow(dY, 2));

		return distance;

	}
	
	
	/**
	 * Returns {@code true} when done.
	 * 
	 * @param x
	 * @param y
	 * @return {@code true} when done.
	 */
	public static boolean isDone(double x, double y) {
		return abs(x - odometer.getX()) < CM_ERR && abs(y - odometer.getY()) < CM_ERR;
	}

	/**
	 * Returns the Navigation Object. Use this method to obtain an instance of
	 * Navigation.
	 * 
	 * @return the Navigation Object
	 */
	public synchronized static Navigation getNavigation() {
		if (nav == null) {
			nav = new Navigation();
		}
		return nav;
	}
	
	/**
	 * Returns interrupt status (has it detected an obstacle or not)
	 * 
	 * @return interrupt
	 * 
	 * */
	public static boolean isInterrupt() {
		return interrupt;
	}

	/**
	 * Sets interrupt state
	 * */
	public static void setInterrupt(boolean interrupt) {
		Navigation.interrupt = interrupt;
	}

}
