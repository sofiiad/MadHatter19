package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import java.io.FileNotFoundException;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import ca.mcgill.ecse211.lab3.LCDInfo;
import ca.mcgill.ecse211.lab3.Log;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

/**
 * ECSE 211 Fall 2019
 * Group 34
 * Lukas Durand & Yi Heng Liu
 */


/**
 * The main class.
 * 
 */
public class Main {
  
  /**
   * Set this to true to print to a file.
   */
  public static final boolean WRITE_TO_FILE = true;
 

  /**
   * Main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) throws InterruptedException {
    
    Navigation navigation = new Navigation(odometer);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];
    
    /* To set up logs
     * 
    Log.setLogging(true, true, false, false);
    Log.setLogWriter(System.currentTimeMillis() + ".log");

    if (WRITE_TO_FILE) {
      setupLogWriter();
    }*/
    
    int buttonChoice;
    new Thread(odometer).start();

    buttonChoice = chooseAvoidanceOrNot();
      
     if (buttonChoice == Button.ID_LEFT) { 
       Thread Navigation = new Thread(navigation);
       Navigation.start();
     }
     else {
       ObstacleAvoidance avoidance = new ObstacleAvoidance(odometer,leftMotor, rightMotor, mediumMotor, usDistance, usData);

       Thread ObstacleAvoidance = new Thread(avoidance);
       ObstacleAvoidance.start();
       new Thread(new UltrasonicPoller()).start();
     }
    
    
    new Thread(new LCDInfo()).start();

    while (Button.waitForAnyPress() != Button.ID_ESCAPE); // do nothing
    
    System.exit(0);
  }

  public static void setupLogWriter() {
    try {
      Log.setLogWriter(System.currentTimeMillis() + ".log");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
  }

  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }
  /**
   * Asks the user whether the robot should navigate or set motors float.
   * 
   * @return the user choice
   */
  private static int chooseNavigationOrFloatMotors() {
    int buttonChoice;
    LCDInfo.showText("< Left | Right >",
                     "       |        ",
                     " Float | Nav on ",
                     "motors |        ",
                     "       |        ");
    
    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }
  
  /**
   * Asks the user whether robot should perform simple navigation or avoidance navigation.
   * 
   * @return the user choice
   */
  private static int chooseAvoidanceOrNot() {
    int buttonChoice;
    LCDInfo.showText("< Left | Right >",
                     "  No   | with   ",
                     "avoid- | avoid- ",
                     " ance  | ance   ",
                     "       |        ");

    do {
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }

  
}
