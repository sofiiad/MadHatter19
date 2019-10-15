
package ca.mcgill.ecse211.lab4;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab4.Resources.*;


/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation {

  /**
   * Returns status of the motors:
   * 
   * True if motors are moving, false if they are not
   */
  public static boolean notNavigating() {
    return (leftMotor.isStalled() && rightMotor.isStalled());
  }
  
  /**
   * Travel to a point by calculating distance and angle needed
   * 
   * @param next_x, next_y
   */
  public static void travelTo (double next_x, double next_y) {

    double current_position_x = odometer.getXYT()[0];
    double current_position_y = odometer.getXYT()[1];

    double delta_x = next_x - current_position_x;      // CO
    double delta_y = next_y - current_position_y;      // CA

    turnTo(getNextTheta(delta_y, delta_x), false);

    // Drive forward to the next position
    // Euclidean distance
    double cord_distance = Math.sqrt((delta_x*delta_x) + (delta_y*delta_y));

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(cord_distance), true);
    rightMotor.rotate(convertDistance(cord_distance), false);
  }

  /**
   * Calculates angles for both left and right turns, turns to the shortest.
   * 
   * @param desired theta, imediate return boolean;
   */
  public static void turnTo (double theta, boolean imediate_return) {

    double initial_ang_odometer = odometer.getXYT()[2];
    double left_turn_ang  = 0;
    double right_turn_ang = 0;

    //Add 360 to the smallest angle and calculate shortest rotation
    if (theta > initial_ang_odometer) {

      //degrees to turn left
      left_turn_ang = Math.abs(initial_ang_odometer - theta);

      initial_ang_odometer += 360;

      //degrees to turn right
      right_turn_ang = Math.abs(initial_ang_odometer - theta);

    } else if (theta < initial_ang_odometer) {

      //degrees to turn right
      right_turn_ang = Math.abs(initial_ang_odometer - theta);

      theta += 360;

      //degrees to turn left
      left_turn_ang = Math.abs(initial_ang_odometer - theta);

    }

    // initialize motors for rotation speed
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    //Either turn left or right
    if (left_turn_ang < right_turn_ang) {

      leftMotor.rotate(-convertAngle(left_turn_ang), true);
      rightMotor.rotate(convertAngle(left_turn_ang), imediate_return);

    } else if (left_turn_ang > right_turn_ang) {

      leftMotor.rotate(convertAngle(right_turn_ang), true);
      rightMotor.rotate(-convertAngle(right_turn_ang), imediate_return);
    }

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
   * @return the wheel rotations necessary to rotate the robot by the angle.
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * Finds the angle the robot needs to head to given delta_x and delta_y
   * 
   * @param delta_y, delta_x
   * @return the angle the robot needs to head to.
   */
  private static double getNextTheta(double delta_y, double delta_x) {

    double ang_x_adjustment = 0;
    double ang_odometer = 0;
    int add_or_sub = 0;

    try {
      // Arctan of abs values will return a value between 0 and pi/2
      ang_x_adjustment = Math.toDegrees(Math.atan(Math.abs(delta_y)/Math.abs(delta_x)));
    } catch (Exception e) {
      ang_x_adjustment = 0; 
    }

    if (delta_x < 0) {            // West
      ang_odometer = 180;

      if (delta_y < 0) {          // 3rd quadrant
        add_or_sub = 1;
      } else if (delta_y > 0) {   // 2nd quadrant
        add_or_sub  = -1;
      }

    } else if (delta_x > 0) {     // East
      ang_odometer = 0;

      if (delta_y < 0) {          // 4th quadrant
        add_or_sub = -1;          
      } else if (delta_y > 0) {   // 1st quadrant
        add_or_sub  = 1;
      }

    } else if (delta_x == 0){

      if (delta_y < 0) {          // South
        ang_odometer = 270;          
      } else if (delta_y > 0) {   // North
        ang_odometer = 90;
      }

    }

    //Defines angle that must be shown in the odometer
    return (ang_odometer + (add_or_sub*ang_x_adjustment));
  }

  /**
   * Error between current point and final point
   * 
   * @param next_x, next_y
   */
  public static double error_from_destination (double next_x, double next_y) {

    double current_position_x = odometer.getXYT()[0];
    double current_position_y = odometer.getXYT()[1];

    double delta_x = next_x - current_position_x;      // CO
    double delta_y = next_y - current_position_y;      // CA

    double error = Math.sqrt((delta_x*delta_x) + (delta_y*delta_y));

    return error;
  }
}
  