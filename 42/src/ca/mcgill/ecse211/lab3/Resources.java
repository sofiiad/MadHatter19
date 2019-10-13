package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access
 * and to avoid cluttering the rest of the codebase. All resources can be
 * imported at once like this:
 * 
 * <p>
 * {@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {

	/**
	 * The different maps our robot could travel on demo day
	 */

	public static int[][] map_1 = { { 1, 3 }, { 2, 2 }, { 3, 3 }, { 3, 2 }, { 2, 1 } };

	public static int[][] map_2 = { { 2, 2 }, { 1, 3 }, { 3, 3 }, { 3, 2 }, { 2, 1 } };

	public static int[][] map_3 = { { 2, 1 }, { 3, 2 }, { 3, 3 }, { 1, 3 }, { 2, 2 } };

	public static int[][] map_4 = { { 1, 2 }, { 2, 3 }, { 2, 1 }, { 3, 2 }, { 3, 3 } };

	public static int[][] map_test = { { 3, 2 }, { 2, 2 }, { 2, 3 }, { 3, 1 } };

	/**
	 * The number of waypoints to travel to
	 */
	public static final int NB_OF_WAYPOINTS = 3;

	/**
	 * The offset distance between the sensor and the wheels in cm.
	 */
	public static final double SENSOR_WHEELS_OFFSET = 1.8;

	/**
	 * The wheel radius in centimeters.
	 */
	public static final double WHEEL_RAD = 2.28; // 2.28 // 3.235

	/**
	 * The robot width in centimeters.
	 */
	public static final double TRACK = 14; // 14 // 14.05

	/**
	 * The speed at which the robot moves forward in degrees per second.
	 */
	public static final int FORWARD_SPEED = 150;

	/**
	 * The speed at which the robot rotates in degrees per second.
	 */
	public static final int ROTATE_SPEED = 150;

	/**
	 * The motor acceleration in degrees per second squared.
	 */
	public static final int ACCELERATION = 500;

	/**
	 * Timeout period in milliseconds.
	 */
	public static final int TIMEOUT_PERIOD = 3000;

	/**
	 * The tile size in centimeters.
	 */
	public static final double TILE_SIZE = 30.48;

	/**
	 * The degree error.
	 */
	public static final double DEG_ERR = 3.0;

	/**
	 * The cm error.
	 */
	public static final double CM_ERR = 2.0;

	/**
	 * The left motor.
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * The right motor.
	 */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	/**
	 * The color sensor.
	 */
	//public static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);

	/**
	 * The ultrasonic sensor.
	 */
	public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));

	/**
	 * The LCD.
	 */
	public static final TextLCD LCD = LocalEV3.get().getTextLCD();

	/**
	 * The odometer.
	 */
	public static Odometer odometer = Odometer.getOdometer();

	/**
	 * The Obstacle Avoidance object.
	 */
	public static ObstacleAvoidance obstacleAvoidance = ObstacleAvoidance.getObstacleAvoidance();

	/**
	 * The navigation.
	 */
	public static Navigation navigation = Navigation.getNavigation();

	/**
	 * The ultrasonic poller.
	 */
	public static UltrasonicPoller usPoller = new UltrasonicPoller();

}
