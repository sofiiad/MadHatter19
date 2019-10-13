package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.LCD;
import static ca.mcgill.ecse211.lab4.Resources.odometer;
import static ca.mcgill.ecse211.lab4.Resources.usPoller;

import java.text.DecimalFormat;

public class Display {

  public static void updateDisplay() {

    LCD.clear();

    // Retrieve x, y and Theta information
    double[] position = odometer.getXYT();

    // Print x,y, and theta information
    DecimalFormat numberFormat = new DecimalFormat("######0.00");
    //LCD.drawString("Edge Option: " + numberFormat.format());
    LCD.drawString("X: " + numberFormat.format(position[0]), 0, 1);
    LCD.drawString("Y: " + numberFormat.format(position[1]), 0, 2);
    LCD.drawString("T: " + numberFormat.format(position[2]), 0, 3);
    LCD.drawString("US Distance: " + numberFormat.format(usPoller.getDistance()), 0, 4);

  }


  /**
   * Shows the text on the LCD, line by line.
   * 
   * @param strings
   *            comma-separated list of strings, one per line
   */
  public static void showText(String... strings) {
    LCD.clear();
    for (int i = 0; i < strings.length; i++) {
      LCD.drawString(strings[i], 0, i);
    }
  }

}
