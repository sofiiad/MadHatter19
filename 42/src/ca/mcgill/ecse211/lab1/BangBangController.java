package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class BangBangController extends UltrasonicController {

  public BangBangController() {
    LEFT_MOTOR.setSpeed(MOTOR_HIGH); // Start robot moving forward
    RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)

    // ********** VARIABLES **********
    int distanceError = distance - Resources.BAND_CENTER;//Computing error b/w US distance and BAND_CENTER

    //******** EVALUATION CASES FOR EV3 IN CORRECT DISTANCE INTERVAL **************
    if(Math.abs(distanceError) <= Resources.BAND_WIDTH) {
      LEFT_MOTOR.setSpeed(MOTOR_HIGH); //Move left motor forward
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH); //Move right motor forward
    }

    //************** EVALUATION CASES FOR EV3 GETTING CLOSER **********************
    //EV3 LEANING INWARDS
    else if(distanceError < 0) {
      if(distanceError > -5 && distanceError <= -10) {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+(MOTOR_LOW/2));
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      }
      //EV3 GETTING CLOSER
      else if (distanceError > -10 && distanceError <= -20) {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+MOTOR_LOW);
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH-MOTOR_LOW);
      }
      //EV3 URGENT RIGHT TURN
      else {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+(MOTOR_LOW));
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+ (int)(1.5*MOTOR_LOW));
      }
    }//End of getting closer evaluations

    //************** EVALUATION CASES FOR EV3 GETTING FARTHER *********************
    else if (distanceError > 0) {
      //EV3 LEANING OUTWARDS
      if(distanceError > 5 && distanceError <=10) {
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+(MOTOR_LOW/2));
        LEFT_MOTOR.setSpeed(MOTOR_HIGH);
      }
      //EV3 GETTING FARTHER
      else if(distanceError > 10 && distanceError <= 20) {
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+MOTOR_LOW);
        LEFT_MOTOR.setSpeed(MOTOR_HIGH-MOTOR_LOW);
      }
      //EV3 URGENT LEFT TURN
      else {
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+(MOTOR_LOW));
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+ (int)(1.5*MOTOR_LOW));
      }//End of getting farther evaluations
      
      //****** SLEEP *********
      //Sleep to give the CPU more time to think about other processes
      try {
        Thread.sleep(50);
      }
      catch(Exception e){
        e.printStackTrace();
      }//End of try catch

    }//End of getting farther evaluations

  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
