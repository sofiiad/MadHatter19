package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
//import lejos.utility.Delay;
//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab4.Resources.*;

/**
* The main driver class
*/
public class Main {
  
  public static int selectedEdge;
  
  /**
  * The main entry point.
  * 
  * @param args
  */  
  public static void main(String[] args) {
    
    selectedEdge = start();
    
    new Thread(odometer).start(); 
    new Thread(new Display()).start();
    new Thread(new SensorPoller()).start();
    new Thread(new Localizer()).start();
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {} // do nothing
    
    System.exit(0);
    
   }
  
  /**
  * Start screen of the program
  */
  public static int start() {
   int selected = 0;
   Display.showText("Start",
                    "left = falling edge",
                    "right = rising edge");
   
   selected = Button.waitForAnyPress();
   LOCAL_STATUS = STATUS.ULTRASONIC;
   
   return selected;
  }
  
  public static void light() {
   
    Button.waitForAnyPress();
    LOCAL_STATUS = STATUS.LIGHT;
    
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
