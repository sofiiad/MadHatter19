package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.LightPoller.startLSPolling;
import static ca.mcgill.ecse211.project.LightPoller.stopLSPolling;
import static ca.mcgill.ecse211.project.Navigation.turn;
import static ca.mcgill.ecse211.project.Navigation.moveForward;
import static ca.mcgill.ecse211.project.Navigation.moveForwardDetection;
import static ca.mcgill.ecse211.project.Navigation.stopMotors;
import static ca.mcgill.ecse211.project.Resources.OFFSET;
import static ca.mcgill.ecse211.project.Resources.US_SENSOR_FRONT;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.UltrasonicPoller.getUsPollerDistance;
import static ca.mcgill.ecse211.project.UltrasonicPoller.startPolling;
import static ca.mcgill.ecse211.project.UltrasonicPoller.stopPolling;

/**
 * Class that offers static methods to detect obstacles situated in a coverage of 180 degrees around the front of the
 * robot and provides detection of lines with avoidance.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class ObstacleAvoidance {

  /**
   * Integer to store the state of obstacles around the robot sense = 0 -> right clear, sense = 1 -> left clear, sense =
   * 2 -> forward clear, sense = 3 -> default no obstacle
   */
  public static int sense = 3;

  /**
   * Returns a boolean list of size 2 with information regarding the location of obstacles to the sides of the robot, if
   * any.
   * 
   * @return objectRight_Left: objectRight_Left[0] == true -> object on the right, objectRight_Left[1] == true -> object
   *         on the left
   */
  public static boolean[] objectsAfterTunnel() {

    boolean objectToTheRight = false;
    boolean objectToTheLeft = false;

    Main.sleepFor(100);
    startPolling(US_SENSOR_FRONT);
    Main.sleepFor(100);

    turn(90); // right turn
    // If distance < 31, there is an object
    if (getUsPollerDistance() < 31) {
      objectToTheRight = true;
    }

    turn(-180); // left turn to opposite direction
    // If distance < 31, there is an object
    if (getUsPollerDistance() < 31) {
      objectToTheLeft = true;
    }

    Navigation.turn(90);

    boolean[] objectRight_Left = {objectToTheRight, objectToTheLeft};
    return objectRight_Left;
  }

  /**
   * Returns a boolean on the presence of an obstacle directly in front of the robot.
   * 
   * @return objectInFront_detected: boolean true if an object in front of the robot is found
   */
  public static boolean objectInFront() {

    boolean objectInFront_detected = false;
    Main.sleepFor(100);
    startPolling(US_SENSOR_FRONT);
    Main.sleepFor(100);

    // If distance < 31, there is an object
    if (getUsPollerDistance() < 31) {
      objectInFront_detected = true;
    }

    return objectInFront_detected;
  }

  /**
   * Method for line detection by light sensors for localization with forward obstacle detection and avoidance.
   * 
   * @return true if no object detected in front of robot before successful line detection.
   */
  public static boolean forwardAvoidanceLineDetection() {
    boolean rightDetected = false;
    boolean leftDetected = false;

    Main.sleepFor(50);
    startLSPolling();
    Main.sleepFor(50);
    startPolling(US_SENSOR_FRONT);
    Main.sleepFor(50);
    moveForwardDetection();

    while (true) {

      // Stop if object detected in front
      if (sense == 2 && getUsPollerDistance() < 25) {
        stopMotors();
        Main.sleepFor(50);
        return false;
      }
      // Advance until a line is detected
      if (LightPoller.leftLS_isLineDetected()) {
        leftMotor.setSpeed(0);
        leftDetected = true;
      }
      if (LightPoller.rightLS_isLineDetected()) {
        rightDetected = true;
        rightMotor.setSpeed(0);
      }
      if (rightDetected && leftDetected) {
        moveForward(OFFSET);
        Main.sleepFor(50);
        stopMotors();
        Main.sleepFor(50);
        rightDetected = false;
        leftDetected = false;
        Main.sleepFor(50);
        stopLSPolling();
        Main.sleepFor(50);
        stopPolling(US_SENSOR_FRONT);
        Main.sleepFor(50);
        return true;
      }
    }
  }
}
