package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.UltrasonicPoller.getUsPollerDistance;
import static ca.mcgill.ecse211.project.UltrasonicPoller.startPolling;
import static ca.mcgill.ecse211.project.UltrasonicPoller.stopPolling;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.US_SENSOR_FRONT;
import static ca.mcgill.ecse211.project.Resources.INTERCEPT_DIST;
import static ca.mcgill.ecse211.project.Resources.WALL_DIST;
import static ca.mcgill.ecse211.project.Resources.MARGIN_DIST;
import static ca.mcgill.ecse211.project.Resources.odometer;

/**
 * Class that offers a static method for localizing the robot's heading using an ultrasonic sensor.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class UltrasonicLocalization {

  /**
   * Boolean to check if US sensor readings are within the noise zone, true after first value below INTERCEP_DIST and
   * MARGIN_DIST is read
   */
  private static boolean inNoiseMargin = false;
  /**
   * Double to store the initial angle within the noise margin.
   */
  private static double initAngle;
  /**
   * Double to store the second angle within the noise margin.
   */
  private static double nextAngle;

  /**
   * Sweeps the robot around the starting corner while polling the US sensor. Sets the odometer theta based on the angle
   * difference between the detection of walls by the US sensor.
   * 
   */
  public static void ultrasonic() {
    // Turn 360 degrees initially counter-clockwise, immediate return true to detect drops in sensor distance
    startPolling(US_SENSOR_FRONT);

    leftMotor.rotate(Navigation.convertAngle(-360), true);
    rightMotor.rotate(Navigation.convertAngle(360), true);


    // While two falling edge distances haven't been detected, rotate one way until it does detect one
    // and then rotate the other way until it detects the second angle
    getWallAngles();

    // First average angle
    double angleA = (initAngle + nextAngle) / 2;

    // Rotate the robot the opposite direction
    leftMotor.setSpeed(ROTATE_SPEED + 40);
    rightMotor.setSpeed(ROTATE_SPEED + 40);
    leftMotor.rotate(Navigation.convertAngle(360), true);
    rightMotor.rotate(Navigation.convertAngle(-360), true);

    // Prevent the US sensor to re-detect the initial wall after turning away from it
    UltrasonicPoller.sleep();

    getWallAngles();

    // Second average angle
    double angleB = (initAngle + nextAngle) / 2;

    double angle_offset = (angleA + angleB) / 2;
    double dt;
    if (angleA > angleB) {
      dt = 45 - angle_offset + 180;
    } else {
      dt = 225 - angle_offset + 180;
    }

    // Set new odometer theta based on perceived angles
    odometer.setTheta(odometer.getT() + dt);

    Main.sleepFor(100);
    // Stop polling the US sensor
    stopPolling(US_SENSOR_FRONT);
  }

  /**
   * Stops the robot once a wall is detected twice while in the noise margin and stores the angles of detection
   */
  private static void getWallAngles() {
    // Discard readings while starting by facing a wall
    while (true) {
      if (getUsPollerDistance() > WALL_DIST) {
        break;
      }
    }

    // Detect falling edge and store angles for computation
    while (true) {
      // First detection falling into noise margin
      if (!inNoiseMargin && getUsPollerDistance() < INTERCEPT_DIST + MARGIN_DIST) {
        inNoiseMargin = true;
        initAngle = odometer.getT();
        // Second detection falling below the noise margin
      } else if (inNoiseMargin && getUsPollerDistance() < INTERCEPT_DIST - MARGIN_DIST) {
        inNoiseMargin = false;
        nextAngle = odometer.getT();

        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        break;
      }
    }
  }

}
