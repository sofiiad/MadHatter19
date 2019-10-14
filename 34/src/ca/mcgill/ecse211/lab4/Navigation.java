package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;


  public class Navigation extends Thread {
    
    /**
     * Navigation class for EV3 
     * Allows robot to navigate from waypoint to waypoint
     */
    


    private Odometer odo;
 
    
    
    //Initial constructor
    public Navigation(Odometer odo) {
        this.odo = odo;
    }

   
    
    //Computes small-angle turn required for robot to head to next waypoint, and turns the robot
    public void turnTo(double theta) {
      
        int wheelRotation;
      
        leftMotor.setSpeed(SLOW);
        rightMotor.setSpeed(SLOW);
        
        //Adjusts theta for minimal turn angle, max angle is +-180
        if(theta < -180) {
          theta += 360; 
        }
        else if(theta > 180 ){
          theta -= 360; 
        }
        try {
           Thread.sleep(500);
         } 
        catch (InterruptedException e) {  
         }
      //Calculates rotation needed based on Track and Wheel radius
        wheelRotation = (int) ((TRACK * theta / 2) / (WHEEL_RADIUS)); 

        leftMotor.rotate(wheelRotation, true);
        rightMotor.rotate(-wheelRotation, false);
    }
    

}

  