package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;
import lejos.utility.Delay;

public class BangBangController extends UltrasonicController {
  
  int filter_count = 0; 

  public BangBangController() {
    LEFT_MOTOR.setSpeed(MOTOR_HIGH); // Start robot moving forward
    RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);
    
    // Lateral distance of robot to closest wall in cm
    int lateral_distance = (int) (distance * Math.cos(ANGLE));
    
    // calculate the error
    int error = BAND_CENTER - lateral_distance;
    
    // filter out spikes when ultra sensor is too close to wall, and when it's turning left
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
    
    // Within range - make it move straight   
    else if (Math.abs(error) <= BAND_WIDTH) {
     
      LEFT_MOTOR.setSpeed(MOTOR_HIGH);
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    } 
    
    // Too close to wall - turn right
    else if (error > BAND_WIDTH) {
      
      LEFT_MOTOR.setSpeed(MOTOR_HIGH + 225); // 225 is a constant for sharp right turn
      RIGHT_MOTOR.setSpeed(MOTOR_LOW);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      filter_count = 0;   
    }
   
    //Too far from the wall - turn left
    else if (error < (-BAND_WIDTH)) {
      
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.setSpeed((int)(MOTOR_HIGH * 0.58)); //0.63 best // 18/32 (cm) = 0.57 value from calculating the radius of the left turn
      RIGHT_MOTOR.forward();
      LEFT_MOTOR.forward();
    } 
  }

  @Override
  public int readUSDistance() {
    return (int)(this.distance);
  }
}
