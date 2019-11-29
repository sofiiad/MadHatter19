package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.LightPoller.*;
import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.ObstacleAvoidance.*;
import static ca.mcgill.ecse211.project.Launcher.releaseLauncher;

/**
 * Class that offers static methods used for localization using the two light sensors at the front of the robot.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class LightLocalization {

  /**
   * Enumeration of possible directions the robot could be facing.
   */
  enum Direction {
    POSITIVE_Y, NEGATIVE_Y, POSITIVE_X, NEGATIVE_X
  };

  /**
   * Stores the direction the robot is facing. Default set to positive X.
   */
  private static Direction direction = Direction.POSITIVE_X;

  /**
   * This method localizes the robot and updates the odometer when it starts in a corner. Turns the robot to face a line
   * after ultrasonic localization
   * 
   * @param corner the corner in which the robot starts in
   */
  public static void light(int corner) {

    // Angle to be set to odometer
    int angle;

    // The starting corner determines the robot's heading after light localization
    if (corner == 0) {
      angle = 90;
    } else if (corner == 1) {
      angle = 0;
    } else if (corner == 2) {
      angle = 270;
    } else {
      angle = 180;
    }

    turnTo(angle);
    Main.sleepFor(30);

    forwardLineDetection();

    turnRight();

    forwardLineDetection();

    // Update odometer after physical sensor readings of tile line
    odometer.setXYT(START[0] * TILE_SIZE, START[1] * TILE_SIZE, angle);
  }

  /**
   * This method performs localization using light sensors reading before entering the tunnel.
   */
  public static void localizeBeforeTunnel() {

    boolean[] wallRight_Left = objectsAfterTunnel();
    if (wallRight_Left[0] == false && wallRight_Left[1] == true) {
      sense = 0;
    } else if (wallRight_Left[0] == true && wallRight_Left[1] == false) {
      sense = 1;
    }

    double theta = odometer.getT();

    direction = getCurrentDirection(theta);

    turn90degrees(sense); // sense=0 means right turn, sense=1 means left turn

    localizeForward();

    // Return to middle of tile
    travelBack(TILE_SIZE / 2);

    sense = turnBack(sense);
    Navigation.turn(sense);

    localizeForward();

    Main.sleepFor(50);
  }

  /**
   * This method performs localization after the robots traverses the tunnel with obstacle avoidance.
   */
  public static void localizeAfterTunnel() { // sense=true --> right, sense=false --> left


    boolean objectInFront = objectInFront();

    if (!objectInFront) { // If there is no object in front of the robot as soon as it gets off the tunel

      Main.sleepFor(50);
      localizeForward();
      Main.sleepFor(50);

      releaseLauncher();

      Main.sleepFor(50);

      boolean[] objectRight_Left = objectsAfterTunnel();

      if (objectRight_Left[0] == false && objectRight_Left[1] == true) {
        sense = 0;
      } else if (objectRight_Left[0] == true && objectRight_Left[1] == false) {
        sense = 1;
      } else if (objectRight_Left[0] == true && objectRight_Left[1] == true) {
        sense = 2;
      } else {
        sense = 3;
      }

      localizeBackward();

      Main.sleepFor(100);
      // sense=3 --> do nothing
      if (sense != 3) {
        turn90degrees(sense); // sense=0 --> right, sense=1 --> left, sense=2 --> forward
        Main.sleepFor(30);

        localizeAvoidanceForward();
      }
    } else {

      boolean[] objectRight_Left = objectsAfterTunnel();

      if (objectRight_Left[0] == false && objectRight_Left[1] == true) {
        sense = 0;
      } else if (objectRight_Left[0] == true && objectRight_Left[1] == false) {
        sense = 1;
      } else {
        sense = 2;
      }

      Main.sleepFor(100);
      // sense=2 --> do nothing
      if (sense == 0 || sense == 1) {
        turn90degrees(sense); // sense=0 --> right, sense=1 --> left
        Main.sleepFor(30);
        localizeForward();
      }
      Main.sleepFor(50);
    }
  }

  /**
   * This method is used to localize after the launching sequence to account for launch kick-back.
   */
  public static void localizeLaunch() {
    localizeForward();

    Main.sleepFor(50);
    // Turn 90 degrees
    turn90degrees();
    Main.sleepFor(50);
    // Travel back twice the light sensor offset to ensure that robot is not over a line
    travelBack(OFFSET * 2);
    Main.sleepFor(50);

    localizeForward();
  }

  /**
   * This method returns the direction the robot is facing
   * 
   * @param theta orientation of robot stored by the odometer
   * @return direction direction the robot is facing (x or y, positive or negative)
   */
  private static Direction getCurrentDirection(double theta) {

    if (theta < 45 || theta > 325) { // facing positive y direction

      direction = Direction.POSITIVE_Y;

    } else if (theta > 135 && theta < 225) { // facing negative y direction

      direction = Direction.NEGATIVE_Y;

    } else if (theta > 45 && theta < 135) { // facing positive x direction

      direction = Direction.POSITIVE_X;

    } else if (theta > 225 && theta < 325) { // facing negative x direction

      direction = Direction.NEGATIVE_X;
    }

    return direction;
  }


  /**
   * This method turns the robot 90 degrees and updates the direction class variable.
   */
  private static void turn90degrees(int sense) {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
      }
    }

    // sense=0 --> right, sense=1 --> left, sense=2 -- do not turn, you are surrounded

    int multiplier = 0; // case where sense=2, multiplier remains 0;
    if (sense == 0) {
      multiplier = 1;
    } else if (sense == 1) {
      multiplier = -1;
    }

    turn(90 * multiplier); // IN case you are surronded, you'll just no turn here

    if (direction == Direction.POSITIVE_Y) {
      direction = Direction.POSITIVE_X;

    } else if (direction == Direction.NEGATIVE_Y) {
      direction = Direction.NEGATIVE_X;

    } else if (direction == Direction.POSITIVE_X) {
      direction = Direction.NEGATIVE_Y;

    } else if (direction == Direction.NEGATIVE_X) {
      direction = Direction.POSITIVE_Y;
    }

  }

  /**
   * Turn back to initial heading after detecting an obstacle
   * 
   * @param sense previous sense value
   * @return sense opposite sense from previous one
   */
  private static int turnBack(int sense) {

    // sense=0 --> right, sense=1 --> left, sense=2 --> do nothing
    if (sense == 0) {
      sense = 1;
    } else if (sense == 1) {
      sense = 0;
    }

    return sense;
  }

  /**
   * This method turns the robot 90 degrees and updates the direction class variable.
   */
  private static void turn90degrees() {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
      }
    }

    turn(90);

    if (direction == Direction.POSITIVE_Y) {
      direction = Direction.POSITIVE_X;
      // System.out.print("\nPositive X");

    } else if (direction == Direction.NEGATIVE_Y) {
      direction = Direction.NEGATIVE_X;
      // System.out.print("\nNegative X");

    } else if (direction == Direction.POSITIVE_X) {
      direction = Direction.NEGATIVE_Y;
      // System.out.print("\nNegative Y");

    } else if (direction == Direction.NEGATIVE_X) {
      direction = Direction.POSITIVE_Y;
      // System.out.print("\nPositive Y");
    }
  }

  /**
   * This method updates the x, y and theta values of the odometer once a robot detects a line.
   * 
   * @param x X coordinate before the robot starts moving forward and sees a line
   * @param y Y coordinate before the robot starts moving forward and sees a line
   */
  private static void updatePosition(double x, double y) {

    int coefficient;

    if (direction == Direction.POSITIVE_Y) { // facing positive y direction

      coefficient = (int) ((y + OFFSET) / TILE_SIZE);
      odometer.setY(coefficient * TILE_SIZE);
      odometer.setTheta(0);

    } else if (direction == Direction.NEGATIVE_Y) { // facing negative y direction

      coefficient = (int) ((y - OFFSET) / TILE_SIZE);
      odometer.setY(coefficient * TILE_SIZE);
      odometer.setTheta(180);


    } else if (direction == Direction.POSITIVE_X) { // facing positive x direction

      coefficient = (int) ((x + OFFSET) / TILE_SIZE);
      odometer.setX(coefficient * TILE_SIZE);
      odometer.setTheta(90);

    } else if (direction == Direction.NEGATIVE_X) { // facing negative x direction

      coefficient = (int) ((x - OFFSET) / TILE_SIZE);
      odometer.setX(coefficient * TILE_SIZE);
      odometer.setTheta(270);
    }
  }

  /**
   * Used for the detection of a line in front of the robot's light sensors for localization.
   */
  private static void forwardLineDetection() {
    boolean rightDetected = false;
    boolean leftDetected = false;

    Main.sleepFor(50);
    startLSPolling();
    Main.sleepFor(50);
    moveForwardDetection();

    while (true) {

      // Advance until a line is detected
      if (LightPoller.leftLS_isLineDetected()) {
        leftMotor.setSpeed(0);
        leftDetected = true;
      }
      if (LightPoller.rightLS_isLineDetected()) {
        rightDetected = true;
        rightMotor.setSpeed(0);
      }

      // Move robot wheel base to the detected line
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
        break;
      }
    }
  }

  /**
   * Detection of a line behind the robot's light sensors for localization.
   */
  private static void backwardLineDetection() {
    boolean rightDetected = false;
    boolean leftDetected = false;

    Main.sleepFor(50);
    startLSPolling();
    Main.sleepFor(50);
    // Move robot backwards
    moveBackwardDetection();

    while (true) {

      // Travel backwards until a line is detected
      if (LightPoller.leftLS_isLineDetected()) {
        leftMotor.setSpeed(0);
        leftDetected = true;
      }
      if (LightPoller.rightLS_isLineDetected()) {
        rightDetected = true;
        rightMotor.setSpeed(0);
      }

      // Move robot wheel base to the detected line
      if (rightDetected && leftDetected) {
        stopMotors();
        Main.sleepFor(50);
        moveForward(OFFSET);
        Main.sleepFor(50);
        rightDetected = false;
        leftDetected = false;
        Main.sleepFor(50);
        stopLSPolling();
        Main.sleepFor(50);
        break;
      }
    }
  }

  /**
   * Method for correcting the odometer with forward line detection.
   */
  public static void localizeForward() {
    // Get current position and direction
    double y = odometer.getY();
    double x = odometer.getX();
    double theta = odometer.getT();
    direction = getCurrentDirection(theta);

    Main.sleepFor(50);
    // Detect forward line
    forwardLineDetection();

    Main.sleepFor(50);
    // Update position
    updatePosition(x, y);
    Main.sleepFor(50);
  }

  /**
   * Method for correcting the odometer with forward line detection and obstacle detection.
   */
  private static void localizeAvoidanceForward() {
    // Get current position and direction
    double y = odometer.getY();
    double x = odometer.getX();
    double theta = odometer.getT();
    direction = getCurrentDirection(theta);

    Main.sleepFor(50);
    // Check if forward line detection was successful
    boolean completed = ObstacleAvoidance.forwardAvoidanceLineDetection();

    Main.sleepFor(50);
    // If no obstacle in front, update position
    if (completed) {
      updatePosition(x, y);
    }
    Main.sleepFor(50);
  }

  /**
   * Method for correcting the odometer with backward line detection
   */
  public static void localizeBackward() {
    // Get current position and direction
    double y = odometer.getY();
    double x = odometer.getX();
    double theta = odometer.getT();
    direction = getCurrentDirection(theta);

    Main.sleepFor(50);
    // Detect backward line
    backwardLineDetection();

    Main.sleepFor(50);
    // Update position
    updatePosition(x, y);
    Main.sleepFor(50);
  }

}
