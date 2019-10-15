package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;

public class Resources {
  
  /**
   * Distance from axis of rotation to Light Sensor
   */
  public static final double AXIS_TO_LIGHT = 12; //best case: 12  
  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.15;
  
  /**
   * The robot width in centimeters.
   */
  public static final double TRACK = 15.15;
  
  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 250; //250
  
  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 150; //150 //75
  
  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 3000;
  
  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;
  
  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor US_SENSOR = 
      new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
  
  /**
   * The light sensor
   */
  public static final EV3ColorSensor LIGHT_SENSOR =
      new EV3ColorSensor(LocalEV3.get().getPort("S4"));
  
  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  
  /**
   * Localization status for switch statement
   */
  public volatile static STATUS LOCAL_STATUS;
  
  /**
   * Status of the localization, used analogously as a simple state machine
   */
  public enum STATUS {ULTRASONIC, LIGHT, FINISHED}
  
  /**
   * Polling sample of ultrasonic sensor
   */
  public static final long US_WAIT = 25;
  
  
  /**
   * Polling sample of light sensor
   */
  public static final long LS_WAIT = 10;
  
  /**
   * Falling edge value in code
   */
  public static final int FALLING_EDGE = Button.ID_LEFT;

  /**
   * Rising edge value in code
   */
  public static final int RISING_EDGE = Button.ID_RIGHT;
  
  /**
   * Threshold to detect rising edge/falling edge
   */
  public static final int THRESHOLD = 40;
  
  /**
   * Noise margin
   */
  public static final int MARGIN = 3;
}
