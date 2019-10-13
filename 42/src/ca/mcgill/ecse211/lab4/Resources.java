package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Navigation;
import ca.mcgill.ecse211.lab4.Odometer;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Resources {

  /**
   * The final destination of the robot. The point (1,1)
   */
  public static int[][] origin = { { 1, 1 } };

  /**
   * The offset distance between the ultrasonic sensor and the wheels in cm.
   */
  public static final double US_SENSOR_WHEELS_OFFSET = 1.8;

  /**
   * The offset distance between the light sensor and the wheels in cm.
   */
  public static final double LS_SENSOR_WHEELS_OFFSET = 12.5; //TODO: calculate

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.28; // 2.28 // 3.235

  /**
   * The robot width in centimeters.
   */
  public static final double TRACK = 13.9; // 14 // 14.05

  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 150;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 100;

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
  public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));

  /**
   * The color sensor.
   */
  public static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);

  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();

  /**
   * The navigation.
   */
  public static Navigation navigation = Navigation.getNavigation();

  /**
   * The ultrasonic poller.
   */
  public static UltrasonicPoller usPoller = new UltrasonicPoller();

  /**
   * The ultrasonic sensor localizer 
   * */
  public static USLocalizer usLocalizer = new USLocalizer();

  /**
   * The light sensor localizer 
   * */
  public static LSLocalizer lsLocalizer = new LSLocalizer();

}
