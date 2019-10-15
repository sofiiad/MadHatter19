package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;
import lejos.utility.Delay;

public class PController extends UltrasonicController {

  private static final int MOTOR_SPEED = 200;
  int deltaSpeed = 5;
  int filter_count = 0;
  
  public int calculateGain(int error, int f1, int f2) {
    return (CHANGE*f1*Math.abs(error)/(f2*2));
  }

  public PController() {
    LEFT_MOTOR.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);

    // TODO: process a movement based on the us distance passed in (P style)
    // Lateral distance of robot to closest wall in cm
    int lateral_distance = (int) (distance * Math.cos(ANGLE));
    
    // calculate the error
    int error = BAND_CENTER - lateral_distance;
    if (Math.abs(error) > BAND_CENTER && filter_count < 60) { // 60 is the number of iterations for which we filter out the spikes
      error = 0;
      filter_count++;
    }
    
    //(Emergency case)
    //Reverse if sensor is too close
    
    if (distance <= (4*BAND_WIDTH)) { //Value taken from readings for when sensor is too close
      
      LEFT_MOTOR.setSpeed((int)(MOTOR_HIGH*(1.5)));
      RIGHT_MOTOR.setSpeed((int)(MOTOR_HIGH*(1.5)));
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.backward();
      Delay.msDelay(600);
      LEFT_MOTOR.setSpeed((int)(MOTOR_HIGH*(1.5)));
      RIGHT_MOTOR.setSpeed((int)(MOTOR_LOW));
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      Delay.msDelay(300);
      filter_count=0;
    }
    
    // Within range
    else if (Math.abs(error) <= BAND_WIDTH) {
        LEFT_MOTOR.setSpeed(MOTOR_SPEED);
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
    } 
    
    // Too close to wall 
    else if (error > BAND_WIDTH) {
      int rightSpeed = MOTOR_SPEED - (calculateGain(error, 2, 1) * deltaSpeed); //2, 1 //Gain values chosen arbitrarily from testing
      int leftSpeed = MOTOR_SPEED + (calculateGain(error, 5, 2) * deltaSpeed); //5, 2
      if (leftSpeed > 400) leftSpeed = 400;  // max speed on left motor
      if (rightSpeed < MOTOR_LOW) rightSpeed = MOTOR_LOW; // min speed on right motor
      
      LEFT_MOTOR.setSpeed(leftSpeed);
      RIGHT_MOTOR.setSpeed(rightSpeed);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      filter_count = 0;
      
      
    } 
    
    // Too far from wall
    else if (error < -(BAND_WIDTH)) {

        int rightSpeed = MOTOR_SPEED + (calculateGain(error, 2, 4)*deltaSpeed); //1, 4
        int leftSpeed = MOTOR_SPEED - (calculateGain(error, 4, 1) * deltaSpeed); //2, 1
        if (error < 1000) {
          rightSpeed = 180; // max right motor speed
          leftSpeed = (int)(rightSpeed * 0.63); // same principle as bang bang
        }
        LEFT_MOTOR.setSpeed(leftSpeed); 
        RIGHT_MOTOR.setSpeed(rightSpeed);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();

    }
  }


  @Override
  public int readUSDistance() {
    return (int) this.distance;
  }

}
