package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class PController extends UltrasonicController {

  private static final int MOTOR_SPEED = 220;
  private static final int SCALE = 100;
  public int distError;
  public double dif;
  public int propConst = 70;
  public double correction;
  public boolean tooClose;
  public boolean steady;
  

  public PController() {
    LEFT_MOTOR.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);

    //: process a movement based on the us distance passed in (P style)
    distError = BAND_CENTER - this.distance; //distance error between set threshold and actual measured distance
    tooClose = (this.distance - MIN_DISTANCE) <= 0; //true if distance less than min-distance
    steady = Math.abs(distError) <= BAND_WIDTH; //true if robot is within band-width threshold
    correction = (int)(propConst * (double)(Math.abs(distError)));
   
    if (distance <= 12) { // an emergency 90 degree turn if object detected
      
      for(int i = 0; i < 2200 ; i++) {
        LEFT_MOTOR.setSpeed(MOTOR_SPEED + 100); // slow left motor
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED + 100); // reverse right motor to pivot away
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
      }
    }
      
    else if (steady == true) { //if robot is within threshold, robot moves straight
      LEFT_MOTOR.setSpeed(MOTOR_SPEED);
      RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
    
    
    else if(tooClose == true) { //if robot is too close to wall
      LEFT_MOTOR.setSpeed((int)(MOTOR_SPEED * 1.2)); // speed up left motor
      RIGHT_MOTOR.setSpeed((int)(150)); // reverse right motor at manual speed
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }
    
    else if(distError > 0) { //if distance error is positive, the robot is too close
      dif = propCalc(distError); //calculates differential
      LEFT_MOTOR.setSpeed(MOTOR_SPEED); //keep left motor high
      RIGHT_MOTOR.setSpeed(Math.abs((int)(MOTOR_SPEED*dif))); //slow right motor with differential
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
    
    else { //if this is called, the error in distance is negative and robot is too far
      dif = propCalc(distError); //calculates differential
      LEFT_MOTOR.setSpeed(Math.abs((int)(MOTOR_SPEED * dif * 0.95)) + 10); //slow left motor
      RIGHT_MOTOR.setSpeed((int)(MOTOR_SPEED + 10)); //keep right motor high
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
  }
public double propCalc(int error) {
  correction = propConst/SCALE + 0.60 + 1/(double)error; //calculates the motor correction based on error
  return correction; //returns correction
}

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
