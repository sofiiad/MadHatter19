package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid 
 * cluttering the rest of the codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */

public class Resources{
  
   /**
    * Angle of ultrasonic sensor tilt on brick
    */
   public static final int ANGLE = 45;
      
   /**
    * Max distance for wall following
    */
   public static final int MAX = 80;
   
    /**
     * Offset from the wall (cm).
     */
    public static final int BAND_CENTER = 14;
    
    /**
     * Width of dead band (cm).
     */
    public static final int BAND_WIDTH = 2;
    
    /**
     * Speed of slower rotating wheel (deg/sec).
     */
    public static final int MOTOR_LOW = 80;
    
    /**
     * Speed of the faster rotating wheel (deg/sec).
     */
    public static final int MOTOR_HIGH = 180;
    
    /**
     * Number of iterations for which we filter out the spikes.
     */
    public static final int FILTER_OUT = 60;
  
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
  public static final int FORWARD_SPEED = 250;
  
  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 150;
  
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
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  
  public static final EV3MediumRegulatedMotor usMotor = 
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor US_SENSOR = 
      new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
  
  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  
  /**
   * Timer at which the robot moves left when it is wall following
   */
  public static int WF_COUNT = 20000;
}