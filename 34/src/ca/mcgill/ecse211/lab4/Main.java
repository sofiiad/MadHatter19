package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import ca.mcgill.ecse211.lab4.LCDInfo;
import ca.mcgill.ecse211.lab4.Navigation;
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
  
  public static boolean MODE;

  /**
   * Main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) throws InterruptedException {
    
    Odometer odo = new Odometer();
    Navigation nav = new Navigation(odometer);
    SampleProvider usDistance = usSensor.getMode("Distance");
    
    float[] usData = new float[usDistance.sampleSize()];
    
    int buttonChoice;
    new Thread(odometer).start();
    
    
    UltrasonicLocalizer USLocalizer = new UltrasonicLocalizer(odo, nav, usDistance, usData, MODE);
    
    buttonChoice = chooseFallingOrRising();
    if (buttonChoice == Button.ID_LEFT) { 
       MODE = false;
    }
    else {
       MODE = true;
    }
    new Thread(new LCDInfo()).start();
    USLocalizer.USRoutine(MODE);

    while (Button.waitForAnyPress() != Button.ID_ESCAPE); // do nothing
    
    LightLocalizer LLocalizer = new LightLocalizer(odo, nav, colorSensor);
    LLocalizer.run();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    
    
    System.exit(0);
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
   * Asks the user whether robot should perform Falling Edge or Rising Edge localization.
   * 
   * @return the user choice
   */
  private static int chooseFallingOrRising() {
    int buttonChoice;
    LCDInfo.showText("< Left | Right >",
                     "       |        ",
                     "  Fal- | Ri-    ",
                     "  ling | sing   ",
                     "       |        ");

    do {
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }

  
}
