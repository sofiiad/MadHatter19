/* ECSE 211 DPM, Group 34
 * Lukas Durand 260716863
 * Yi Heng Liu 260847324
 */
package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class BangBangController extends UltrasonicController {

  public int distError; // distance error
  public boolean tooClose; // true if robot is too close to wall
  public boolean steady; // true if robot is within the correct distance threshold
  
  public BangBangController() {
    LEFT_MOTOR.setSpeed(MOTOR_HIGH); // Start robot moving forward
    RIGHT_MOTOR.setSpeed(MOTOR_HIGH); 
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  // process a movement based on the us distance passed in (BANG-BANG style)
  
  public void processUSData(int distance) {
    filter(distance);
    distError = BAND_CENTER - this.distance; //distance error between set threshold and actual measured distance
    tooClose = (this.distance-MIN_DISTANCE) <= 0; //true if distance less than min-distance
    steady = Math.abs(distError) <= BAND_WIDTH; //true if robot is within band-width threshold
    
    if (this.distance <= 12) { // an emergency 90 degree turn if object detected
      for(int i = 0; i < 1700; i++) {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH + 100); // increase left motor
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH + 100); // reverse right motor to pivot away
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
      }
    }
    if (steady == true) { //if robot is within threshold, robot moves straight
      LEFT_MOTOR.setSpeed(MOTOR_HIGH + 50);
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH + 50);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
    else if (tooClose == true) { //if robot is too close to wall
      LEFT_MOTOR.setSpeed(MOTOR_HIGH + 25); // speed up left motor
      RIGHT_MOTOR.setSpeed(MOTOR_LOW + 25); // reverse right motor to pivot away
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }
    
    else if (distError > 0) { //if distance error is positive, the robot is too close
      LEFT_MOTOR.setSpeed(MOTOR_LOW); //keep left motor high
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH + 20); //slow right motor
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
    else { //if this is called, the error in distance is negative and robot is too far
      LEFT_MOTOR.setSpeed(MOTOR_LOW); //slow left motor
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH + 10); //keep right motor high
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
   
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
