package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.LightPoller.*;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.OFFSET;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.CORRECTION_PERIOD;

/**
 * Class that offers static methods to correct the odometer based on knowledge of where the robot should be heading and
 * the detection of tile lines by the light sensors.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class OdometryCorrection {

  /**
   * Long variable to store correction start and end times.
   */
  private static long correctionStart, correctionEnd;

  /**
   * Stores the new theta to be set to odometer
   */
  private static double correctionTheta = 0;

  /**
   * Boolean storing the state of the correction
   */
  public static boolean isCorrecting = false;

  /**
   * Method for odometry correction with two front light sensors
   * 
   * @param correctedX to be set as X
   * @param correctedY to be set as Y
   */
  public synchronized static void correct(double correctedX, double correctedY) {

    boolean rightDetected = false;
    boolean leftDetected = false;
    boolean correctOdometry = true;
    isCorrecting = false;
    Main.sleepFor(50);
    Navigation.moveForward(50, true);
    // Move forward until a line is detected
    while (correctOdometry) {
      correctionStart = System.currentTimeMillis();

      // When left sensor detected, stop left motor
      if (leftLS_isLineDetected()) {
        leftMotor.setSpeed(0);
        leftDetected = true;
        isCorrecting = true;
      }
      // When right sensor detected, stop right motor
      if (rightLS_isLineDetected()) {
        rightMotor.setSpeed(0);
        rightDetected = true;
        isCorrecting = true;
      }

      // Once both motors stopped, set corrected position based on map knowledge
      if (leftDetected && rightDetected) {
        Navigation.moveForward(OFFSET);
        double orientation = odometer.getT();
        // Correct theta based on perceived heading
        if (orientation < 45 || orientation > 325) {
          correctionTheta = 0;
        } else if (orientation > 45 && orientation < 135) {
          correctionTheta = 90;
        } else if (orientation > 135 && orientation < 225) {
          correctionTheta = 180;
        } else if (orientation < 325 && orientation > 225) {
          correctionTheta = 270;
        }
        // Set corrected position
        odometer.setXYT(correctedX, correctedY, correctionTheta);
        isCorrecting = false;
        correctOdometry = false;
      }

      // Correction sleep period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }
}
