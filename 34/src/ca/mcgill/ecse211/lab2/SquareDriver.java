package ca.mcgill.ecse211.lab2;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab2.Resources.*;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class SquareDriver {

  /**
   * Drives the robot in a square of size 3x3 Tiles. It is to be run in parallel
   * with the odometer and odometer correction classes to allow testing their functionality.
   */
  public static void drive() {
    // spawn a new Thread to avoid this method blocking
    (new Thread() {
      public void run() {
        // reset the motors
        leftMotor.stop();
        rightMotor.stop();
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);


        // Sleep for 2 seconds
        Main.sleepFor(TIMEOUT_PERIOD);

        for (int i = 0; i < 4; i++) {
          // drive forward three tiles
          leftMotor.setSpeed(FORWARD_SPEED);
          rightMotor.setSpeed(FORWARD_SPEED);

          leftMotor.rotate(convertDistance(3 * TILE_SIZE), true);
          rightMotor.rotate(convertDistance(3 * TILE_SIZE), false);

          // turn 90 degrees clockwise
          leftMotor.setSpeed(ROTATE_SPEED);
          rightMotor.setSpeed(ROTATE_SPEED);

          leftMotor.rotate(convertAngle(90.0), true);
          rightMotor.rotate(-convertAngle(90.0), false);
        }
      }
    }).start();
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
}
