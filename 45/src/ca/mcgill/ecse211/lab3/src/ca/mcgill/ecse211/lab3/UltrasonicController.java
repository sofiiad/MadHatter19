package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;

/**
 * Controller that controls the robot's movements based on ultrasonic data.
 */
public abstract class UltrasonicController {

  int distanceUS;
  int filterControl;
  
  /**
   * Perform an action based on the US data input.
   * 
   * @param distance the distance to the wall in cm
   */
  public abstract void processUSData(int distanceUS);

  /**
   * Returns the distance between the US sensor and an obstacle in cm.
   * 
   * @return the distance between the US sensor and an obstacle in cm
   */
  public abstract int readUSDistance();
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * @param distance distance in cm
   */
  void filter(int distanceUS) {
    if (distanceUS >= MAX && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the filter value
      filterControl++;
    } else if (distanceUS >= MAX) {
      // Repeated large values, so there is nothing there: leave the distance alone
      this.distanceUS = distanceUS;
    } else {
      // distance went below MAX: reset filter and leave distance alone.
      filterControl = 0;
      this.distanceUS = distanceUS;
    }
  }
  
}