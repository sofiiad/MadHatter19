package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import static ca.mcgill.ecse211.lab4.Navigation.*;
import static ca.mcgill.ecse211.lab4.SensorPoller.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.utility.Delay;

/**
 * 
 * State machine that performs ultrasonic localizer and line localizer.
 * 
 * @author Karl Koerich
 * @author Alex Choi
 */

public class Localizer extends Thread{

  
  public void run() {
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.stop();
    rightMotor.stop();
    Delay.msDelay(3000);
    

    int prev_distance = 0; // Record previous distance so we detect spikes in distance changes
    if (Main.selectedEdge == RISING_EDGE) {
      prev_distance = 500;
    }
    int current_distance = 0;
    double[] two_angles = new double[2];
    
    while (true) {
      switch(LOCAL_STATUS) {
        case ULTRASONIC:
          int angle_detection = 0;
          
          // Turn 360 degrees initially counter-clockwise
          leftMotor.rotate(Navigation.convertAngle(-360), true);
          rightMotor.rotate(Navigation.convertAngle(360), true);
          
          // While two falling edge distances haven't been detected, rotate one way until it does detect one
          // and then rotate the other way until it detects the second angle
          while (angle_detection < 2) {

            // rising edge and falling edge implementation of US localizer, principle is the same.
            switch(Main.selectedEdge) {
              
              case FALLING_EDGE:
                current_distance = SensorPoller.distance;
                if (current_distance < (THRESHOLD - MARGIN) && prev_distance > THRESHOLD) {
                  
                  rightMotor.stop();
                  leftMotor.stop();
                  
                  two_angles[angle_detection] = odometer.getXYT()[2];
                  angle_detection++;
                }
                
                // rotate the robot the other way around (clockwise), do it only once
                if (angle_detection == 1 && !Navigation.notNavigating()) {
                  // Turn 360 degrees
                  leftMotor.setSpeed(ROTATE_SPEED);
                  rightMotor.setSpeed(ROTATE_SPEED);
                  leftMotor.rotate(Navigation.convertAngle(360), true);
                  rightMotor.rotate(Navigation.convertAngle(-360), true);
                }

                break;

              case RISING_EDGE:
                current_distance = SensorPoller.distance;
                if (current_distance > (THRESHOLD + MARGIN) && prev_distance < THRESHOLD) {
                  
                  rightMotor.stop();
                  leftMotor.stop();
                  
                  two_angles[angle_detection] = odometer.getXYT()[2];
                  angle_detection++;
                }
                
                // rotate the robot the other way around (clockwise), do it only once
                if (angle_detection == 1 && !Navigation.notNavigating()) {
                  // Turn 360 degrees
                leftMotor.setSpeed(ROTATE_SPEED);
                rightMotor.setSpeed(ROTATE_SPEED);
                leftMotor.rotate(Navigation.convertAngle(360), true);
                rightMotor.rotate(Navigation.convertAngle(-360), true);
                }


                break;
              default:
                System.out.println("ERROR");
                System.exit(1);
            }
            prev_distance = current_distance;
          }
          
          
          double angle_offset = (two_angles[0] + two_angles[1])/2;
          
          double dt;
          if ((two_angles[1] < two_angles[0] && Main.selectedEdge == FALLING_EDGE) || (two_angles[0] < two_angles[1] && Main.selectedEdge == RISING_EDGE)) {
            dt = 45 - angle_offset - 90;
          } else {
            dt = 225 - angle_offset - 90;
          }
          odometer.setTheta(odometer.getXYT()[2] + dt);
          Delay.msDelay(1000);
          Navigation.turnTo(0, false);
          
          Main.light();
          
          break;
        case LIGHT:
          
          odometer.setTheta(90);
          
          //At this point, we assume the angle theta has already been corrected in the odometer.
          
          //Turns towards the (0, 0) coordinate
          Navigation.turnTo(45, false);
          
          leftMotor.setSpeed(FORWARD_SPEED);
          rightMotor.setSpeed(FORWARD_SPEED);
          leftMotor.rotate(270, true);
          rightMotor.rotate(270, false);
          leftMotor.stop();
          rightMotor.stop();

          //Start rotating around, counter-clockwise

          leftMotor.setSpeed(ROTATE_SPEED);
          rightMotor.setSpeed(ROTATE_SPEED);


          //Rotates 360 degrees counter-clockwise
          leftMotor.rotate(convertAngle(-360), true);
          rightMotor.rotate(convertAngle(360), true); //We want to detect the points where the robot crosses the lines

          // Vector that will hold the angles
          int pointer = 0;
          double angles_ccw[] = new double[4];

          // Light sensor to detects line and stores angles into the vector.
          while (pointer < 4) {

            if (intensity < 280) { // Safe value for line detecting.

              double angle = odometer.getXYT()[2];
              
              if (pointer == 0) {
                angles_ccw[pointer] = angle;
                pointer++;
              }
              else if (Math.abs((angle - angles_ccw[pointer-1])) > 10) {
                angles_ccw[pointer] = angle;
                pointer++;

              }
            }
          }
         
          // At this point, we should have the 4 angles for when the lines were detected

          double theta_x = (Math.min(angles_ccw[1], angles_ccw[3]) + 360) - (Math.max(angles_ccw[1], angles_ccw[3]));
          double theta_y = (Math.min(angles_ccw[0], angles_ccw[2]) + 360) - (Math.max(angles_ccw[0], angles_ccw[2]));
          
          // Use guide in the slides to find x and y distance from point (1, 1)

          double x = - Math.abs(AXIS_TO_LIGHT*Math.cos(Math.toRadians(theta_y/2)));
          double y = - Math.abs(AXIS_TO_LIGHT*Math.cos(Math.toRadians(theta_x/2)));

        
          // Correct odometer based on calculations

          odometer.setX(x);
          odometer.setY(y);

          // Travel to origin and face 0 degrees axis
          Button.waitForAnyPress();

          travelTo(0, 0);
          turnTo(90, false);
          
          LOCAL_STATUS = STATUS.FINISHED;
          
          break;
          
        case FINISHED:

          Sound.beep();
          Button.waitForAnyPress();
          Sound.beep();
          System.exit(0);
          break;
        
        default:
          
          System.out.println("ERROR");
          Button.waitForAnyPress();
          Sound.beep();
          System.exit(1);

      }
    }
  }
}
