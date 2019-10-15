
package ca.mcgill.ecse211.lab3;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;
import lejos.utility.Delay;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation extends UltrasonicController {

  // Variables for wall following 
  private static int lateral_distance;
  private static int error_wall;
  private static int distanceUS = 250;
  private static int filter_count = 0;
  public static boolean Navigating;
  
  // Variables for navigation
  private static final double[][] sequence_positions = {{1,3},{2,2},{3,3},{3,2},{2,1}};
  private static final int length = sequence_positions.length; //Length of list of positions
  
  /**
   * Drives the robot in the sequence of predetermined coordinates.
   */
  public static void drive() {
    // spawn a new Thread to avoid this method blocking`12
    (new Thread() {
      public void run() {
        
        // reset the motors
        leftMotor.stop();
        rightMotor.stop();
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);

        // Sleep for 2 seconds
        Main.sleepFor(TIMEOUT_PERIOD);
        
        
        for (int i_sequence = 0; i_sequence < length;  i_sequence++)
        { 
          Navigating = false;
          
          double[] next_position = sequence_positions[i_sequence]; // Coordinates as Integers
          double next_x = (next_position[0]*TILE_SIZE);
          double next_y = (next_position[1]*TILE_SIZE);
          
          double error = 100; // 100: Arbitrary number for larger, unacceptable error
          
          while (error > 2) { // 2: Arbitrary number for acceptable error
                        
            Navigating = true;
            travelTo(next_x, next_y);
                      
            //Wall Following if it detects something
            
            int count_wall_following = 2*WF_COUNT; // Arbitrary large number
            
            while (leftMotor.isMoving() && rightMotor.isMoving()) {
              
              if (distanceUS < BAND_CENTER) {
                leftMotor.stop();
                rightMotor.stop();
                leftMotor.forward();
                rightMotor.forward();
                leftMotor.setSpeed(ROTATE_SPEED);
                rightMotor.setSpeed(ROTATE_SPEED);
                leftMotor.rotate(convertAngle(90), true);
                rightMotor.rotate(-convertAngle(90), false);
                rotateUS();
                count_wall_following = 0;
              }
  
              while (count_wall_following < WF_COUNT) { 
                
                // Emergency case - Reverse if sensor is too close 
                if (distanceUS <= (4*BAND_WIDTH)) { //Value taken from readings for when sensor is too close
                  
                  leftMotor.setSpeed((int)(MOTOR_HIGH*(1.5)));
                  rightMotor.setSpeed((int)(MOTOR_HIGH*(1.5)));
                  leftMotor.backward();
                  rightMotor.backward();
                  Delay.msDelay(600);
                  leftMotor.setSpeed((int)(MOTOR_HIGH*(1.5)));
                  rightMotor.setSpeed((int)(MOTOR_LOW));
                  leftMotor.forward();
                  rightMotor.forward();
                  Delay.msDelay(300);
                  filter_count=0;
                }
                
                // Within range - make it move straight   
                else if (Math.abs(error_wall) <= BAND_WIDTH) {
                 
                  leftMotor.setSpeed(MOTOR_HIGH);
                  rightMotor.setSpeed(MOTOR_HIGH);
                  leftMotor.forward();
                  rightMotor.forward();
                } 
                
                // Too close to wall - turn right
                else if (error_wall > BAND_WIDTH) {
                  
                  leftMotor.setSpeed(MOTOR_HIGH + 225); // 225: Constant for sharp right turn
                  rightMotor.setSpeed(MOTOR_LOW);
                  leftMotor.forward();
                  rightMotor.forward();
                  filter_count = 0;   
                }
               
                // Too far from the wall - turn left
                else if (error_wall < (-BAND_WIDTH)) {
                  
                  rightMotor.setSpeed(MOTOR_HIGH);
                  leftMotor.setSpeed((int)(MOTOR_HIGH * 0.61)); // Value from calculating the radius of the left turn
                  rightMotor.forward();
                  leftMotor.forward();
                  count_wall_following++;
                }
                
              }
              
              // When it reaches a considerable number of left turns (wall following)
              if (count_wall_following == WF_COUNT) {
                leftMotor.stop();
                rightMotor.stop();
                rotateBackUS();
                count_wall_following = 2*WF_COUNT; // Resets arbitrary large number
                break;
              }
              
            }
            
            // Recalculates the error to stay inside loop and call travelTo in case its necessary 
            error = error_from_destination(next_x, next_y);
            
          }
          
          Navigating = false;
        }
      }
    }).start();
  }
  
  /** 
   * @return the distance between the US sensor and an obstacle in cm
   */
  public int readUSDistance() {
    return distanceUS;
  }
  
  /**
   * Perform an action based on the US data input.
   * @param distance the distance to the wall in cm
   */
  public void processUSData(int distance) {
    distanceUS = distance;
    //Filter out peaks and bad reading and saves value to this.distanceUS
    filter(distance);
    
    // Lateral distance of robot to closest wall in cm
    lateral_distance = (int) (distanceUS * Math.cos(ANGLE));
    
    // calculate the error
    error_wall = BAND_CENTER - lateral_distance;
    
    // filter out spikes when ultra sensor is too close to wall, and when it's turning left
    if (Math.abs(error_wall) > BAND_CENTER && filter_count < FILTER_OUT) {
      error_wall = 0;
      filter_count++;      
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
    
    turnTo(getNextTheta(delta_y, delta_x));
       
    // Drive forward to the next position
    //Euclidian distance
    
    double cord_distance = Math.sqrt((delta_x*delta_x) + (delta_y*delta_y));
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(cord_distance), true);
    rightMotor.rotate(convertDistance(cord_distance), true);
  }
  
  /**
   * Calculates angles for both left and right turns, turns to the shortest.
   * 
   * @param desired theta;
   */
  private static void turnTo (double theta) {
    
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
      
      //turnTo(left_turn_ang);
      
      leftMotor.rotate(-convertAngle(left_turn_ang), true);
      rightMotor.rotate(convertAngle(left_turn_ang), false);
      
    } else if (left_turn_ang > right_turn_ang) {
      
      leftMotor.rotate(convertAngle(right_turn_ang), true);
      rightMotor.rotate(-convertAngle(right_turn_ang), false);
    }
    
  }
  
  
  /**
   * Turns the US sensor 45 degrees when entering the Wall following algorithm
   */
  public static void rotateUS() {
    usMotor.rotate(-ANGLE);
  }
  
  public static void rotateBackUS() {
    usMotor.rotate(ANGLE);
  }
  
  public boolean isNavigating() {
    return Navigating;
  }
}

