// Lab3.java
package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.utility.Delay;
// static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;
import ca.mcgill.ecse211.lab3.UltrasonicController;

/**
 * The main driver class for the odometry lab.
 */
public class Main {
  
  public static UltrasonicController selectedController;

  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {
    start();
    //new Thread(odometer).start();
    //new Thread(new UltrasonicPoller()).start();
    
    Navigation.drive();
    
    new Thread(odometer).start();
    new Thread(new UltrasonicPoller()).start();
    
    new Thread(new Display()).start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing
    
    System.exit(0);
  }
  
  /**
   * Start screen of the program
   */
  public static void start() {
    Display.showText("     Start?     ",
                     "Press any button",
                     "    to start    ");
    Button.waitForAnyPress();
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
  
}
