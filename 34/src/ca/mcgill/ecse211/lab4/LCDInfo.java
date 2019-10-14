package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.lab4.UltrasonicLocalizer;
import lejos.utility.TimerListener;

/**
 * Class to display information on the LCD.
 */
public class LCDInfo implements Runnable {
  
  private UltrasonicLocalizer USLocalizer;
  private double[] position;
  private float USDistance = 0;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;
  private Odometer odo;
  double A =0;
  double B = 0;
  


  public void run() {
    
    LCD.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();
      

      // Retrieve X, Y, Theta, and USPoller distance information
      position = odometer.getXYT();
      A = USLocalizer.getA();
      B = USLocalizer.getB();
      USDistance = UltrasonicLocalizer.getFilteredDist();
      
      // Print X, Y, Theta, and USPoller distance information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      //LCD.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      //LCD.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      LCD.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      LCD.drawString("US Distance: " + numberFormat.format(USDistance), 0, 3);
      LCD.drawString("A: " + numberFormat.format(A), 0, 4);
      LCD.drawString("B: " + numberFormat.format(B), 0, 5);
      
      // Ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }
  
  /**
   * Sets the timeout in ms.
   * 
   * @param timeout
   */
  public void setTimeout(long timeout) {
    this.timeout = timeout;
  }
  
  /**
   * Shows the text on the LCD, line by line.
   * 
   * @param strings comma-separated list of strings, one per line
   */
  public static void showText(String... strings) {
    LCD.clear();
    for (int i = 0; i < strings.length; i++) {
      LCD.drawString(strings[i], 0, i);
    }
  }
}
