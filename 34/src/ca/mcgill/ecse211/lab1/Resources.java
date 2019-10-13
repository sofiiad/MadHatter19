package ca.mcgill.ecse211.lab1;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * Class for static resources (things that stay the same throughout the entire program execution),
 * like constants and hardware.
 * 
 * Use these resources in other files by adding this line at the top (see examples):<br>
 * 
 * {@code import static ca.mcgill.ecse211.lab1.Resources.*;}
 */
public class Resources {
  //Parameters: adjust these for desired performance
  
  /**
   * Offset from the wall (cm).
   */
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
   * The LCD screen used for displaying text.
   */
  public static final TextLCD TEXT_LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor US_SENSOR = 
      new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
}
