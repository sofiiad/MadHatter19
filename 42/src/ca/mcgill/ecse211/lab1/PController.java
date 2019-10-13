package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class PController extends UltrasonicController {

  private static final int MOTOR_SPEED = 200;

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
    //******** VARIABLES ***********
    int distanceError = distance - Resources.BAND_CENTER; //Computing error b/w US distance and BAND_CENTER
    int gain = proportionalGain(distanceError); //Declaring proportion variable 

    //******** EVALUATION CASES FOR EV3 IN CORRECT DISTANCE INTERVAL **************
    if(Math.abs(distanceError) <= Resources.BAND_WIDTH) {
      LEFT_MOTOR.setSpeed(MOTOR_HIGH); //Move left motor forward
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH); //Move right motor forward
    }

    //**************** EVALUATION CASES FOR EV3 GETTING CLOSER ********************
    //EV3 LEANING INWARDS
    else if(distanceError < 0) {
      if(distanceError > -5 && distanceError <= -10) {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+ (2*gain));
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      }
      //EV3 GETTING CLOSER
      else if (distanceError > -10 && distanceError <= -20) {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+gain);
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH-gain);
      }
      //EV3 URGENT RIGHT TURN
      else {
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+gain);
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+ (int)(1.5*gain));
      }
    }//End of getting closer evaluations

    //**************** EVALUATION CASES FOR EV3 GETTING FARTHER ********************
    else if(distanceError > 0) {
      if(distanceError > 5 && distanceError <=10) {
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+ (2*gain));
        LEFT_MOTOR.setSpeed(MOTOR_HIGH);
      }
      //EV3 GETTING FARTHER
      else if (distanceError > 10 && distanceError <= 20) {
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+gain);
        LEFT_MOTOR.setSpeed(MOTOR_HIGH-gain);
      }
      //EV3 URGENT LEFT TURN
      else {
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH+gain);
        LEFT_MOTOR.setSpeed(MOTOR_HIGH+ (int)(1.5*gain));
      }
    }//End of getting farther evaluation

    //****** SLEEP *********
    //Sleep to give the CPU more time to think about other processes
    try {
      Thread.sleep(50);
    }
    catch(Exception e){
      e.printStackTrace();
    }//End of try catch
    
  }


  //************ GAIN METHOD **************
  //Method which will be used to compute the gain proportional to the distance error
  int proportionalGain (int diff) {
    int correction = (int)(2.5* diff);
    if (correction >= 50) { //Setting a limit to the gain
      return 50;
    }
    else {
      return Math.abs(correction);
    }
  }//End of proportionalGain method

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
