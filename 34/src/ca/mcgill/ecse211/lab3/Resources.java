package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid 
 * cluttering the rest of the codebase.
 */
public class Resources {
  
 public static final int BAND_CENTER = 28;
  
  /**
   * Width of dead band (cm).
   */ 
  public static final int BAND_WIDTH = 4;
  
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_LOW = 140;
  
  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 240;
  
  public static final int FILTER_OUT = 20;
  
  public static final int MIN_DISTANCE = 22;

  /**
   * The wheel radius.
   */
  public static final double WHEEL_RADIUS = 2.125;
  
  /**
   * The robot width.
   */
  public static final double TRACK = 14.63;
  
  /**
   * The left radius.
   */
  public static final double LEFT_RADIUS = 2.75;
  
  /**
   * The right radius.
   */
  public static final double RIGHT_RADIUS = 2.75;
  
  /**
   * The width.
   */
  public static final double WIDTH = 15.8;
  
  /**
   * The odometer timeout period.
   */
  public static final int TIMEOUT_PERIOD = 50;
  
  /**
   * The fast speed.
   */
  public static final int FAST = 200;
  
  /**
   * The slow speed.
   */
  public static final int SLOW = 120;
  
  /**
   * The acceleration.
   */
  public static final int ACCELERATION = 4000;
 
  /**
   * The cm size of the tile.
   */
  public static final double TILE_SIZE = 30.48;
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
  
  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
  
  /**
   * The small motor.
   */
  public static final EV3MediumRegulatedMotor mediumMotor = new EV3MediumRegulatedMotor(MotorPort.C);
  
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The ultrasonic poller.
   */
  public static UltrasonicPoller usPoller = new UltrasonicPoller();
  
  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();

}
