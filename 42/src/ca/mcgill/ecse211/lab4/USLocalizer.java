package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;

import lejos.hardware.Button;
import lejos.hardware.Sound;

public class USLocalizer {

  /*
   * Distance from the wall used to detect back wall and left wall angles
   */
  private static final double WALL_THRESHOLD = 32;

  /*
   * Noise margin constant
   */
  private static final double ERROR = 5;

  /**
   * Method to localize the EV3 by applying the rising edge position calculation
   * technique
   */
  public void risingEdge() {

    double angle1; // Angle at which the back wall is detected
    double angle2; // Angle at which the left wall is detected

    turnToRisingEdge(false);
    // Turn clockwise to detect back wall angle
    turnToRisingEdge(true);
    angle1 = odometer.getTheta(); // Record back wall angle
    //System.out.print("THETA: " + angle1);

    // Turn counter-clockwise to detect left wall angle
    turnToRisingEdge(false);
    angle2 = odometer.getTheta(); // Record left wall angle
    //System.out.print("THETA: " + angle2);

    thetaLocalization(angle1, angle2, true);

  }

  /**
   * Method to localize the EV3 by applying the falling edge position calculation
   * technique
   */
  public void fallingEdge() {

    double angle1; // Angle at which the back wall is detected
    double angle2; // Angle at which the left wall is detected


    turnToFallingEdge(false);		

    // Turn clockwise to detect back wall angle
    turnToFallingEdge(true);
    angle1 = odometer.getTheta(); // Record back wall angle

    // Turn counter-clockwise to detect left wall angle
    turnToFallingEdge(false);
    angle2 = odometer.getTheta(); // Record left wall angle

    thetaLocalization(angle1, angle2, false);

  }

  /**
   * Method to calculate the angle correction (angle to be added to the heading
   * reported by the odometer to orient the robot correctly) and turn the robot to
   * face 0 degrees.
   * 
   * @param angleAlpha
   *            Angle at which the back wall is detected
   * @param angleBeta
   *            Angle at which the left wall is detected
   * @param risingEdge
   *            Flag for using rising or falling edge method
   */
  private void thetaLocalization(double angleAlpha, double angleBeta, boolean risingEdge) {

    double deltaT = 0;

    // Calculate the theta correction
    if (angleAlpha < angleBeta) {
      deltaT = 45 - (angleAlpha + angleBeta) / 2;
    } else if (angleAlpha > angleBeta) {
      deltaT = 225 - (angleAlpha + angleBeta) / 2;
    }

    if (!risingEdge) {
      deltaT += 180;
    }

    Display.updateDisplay();

    // Set odometer theta to right angle
    odometer.setTheta(odometer.getTheta() + deltaT);

    Display.updateDisplay();

    // Rotate to face 0 degrees
    Navigation.turnTo(0);
    Display.updateDisplay();
  }

  /**
   * This method makes the robot turn until it detects a rising edge
   * 
   * @param clockwise
   *            True for turning clockwise, false for counter-clockwise
   */
  private void turnToRisingEdge(boolean clockwise) {

    // start turning
    Navigation.rotate(clockwise);
    double prevDistance, currDistance;

    boolean turning = true;

    currDistance = usPoller.getDistance();
    prevDistance = currDistance;

    LCD.clear();
    while (turning) {

      // Get current distance
      currDistance = usPoller.getDistance();

      if (currDistance >= WALL_THRESHOLD + ERROR && prevDistance <= WALL_THRESHOLD) { // rising edge detected

        Display.updateDisplay();
        Navigation.stopMotors();
        Sound.beep();
        turning = false;
      }

      // Store previous distance
      prevDistance = currDistance;

    }

  }

  /**
   * This method makes the robot turn until it detects a falling edge
   * 
   * @param boolean
   *            clockwise True for turning clockwise, false for counter-clockwise
   */
  private void turnToFallingEdge(boolean clockwise) {

    // start turning
    Navigation.rotate(clockwise);
    double prevDistance, currDistance;

    boolean turning = true;

    currDistance = usPoller.getDistance();
    prevDistance = currDistance;

    while (turning) {

      // Get current distance
      currDistance = usPoller.getDistance();
      //System.out.println(prevDistance + "	" + currDistance);

      if (currDistance <= WALL_THRESHOLD - ERROR 
          && prevDistance >= WALL_THRESHOLD + ERROR) { // falling edge detected				
        Display.updateDisplay();
        Navigation.stopMotors();
        Sound.beep();
        turning = false;
      } 

      // Store previous distance
      prevDistance = currDistance;

    }
  }
}
