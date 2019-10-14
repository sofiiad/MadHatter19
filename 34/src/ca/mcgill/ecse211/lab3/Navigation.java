package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import static java.lang.Math.*;


  public class Navigation extends Thread {
    
    /**
     * Navigation class for EV3 
     * Allows robot to navigate from waypoint to waypoint
     */
    
    /*
    * Maps for navigation
    */    
    private int[][] waypoints = {{2,1},{1,1},{1,2},{2,0}}; //MAP1
    //private int[][] waypoints = {{0, 2}, {1 ,1}, {2, 2}, {2, 1}, {1, 0}}; //MAP2
    //private int[][] waypoints = {{1, 1}, {0 ,2}, {2, 2}, {2, 1}, {1, 0}}; //MAP3
    //private int[][] waypoints = {{1, 0}, {2 ,1}, {2, 2}, {0, 2}, {1, 1}}; //MAP4
    //private int[][] waypoints = {{0, 1}, {1 ,2}, {1, 0}, {2, 1}, {2, 2}};   //MAP5
    //private int[][] waypoints = {{0, 2}, {2 ,2}, {2, 0}, {0, 0}}; //TESTMAP

    private Odometer odo;
    private double[] pos = new double[3]; //To store current position from odometer
    
    //Variables for travelTo()
    private double dX; 
    private double dY; 
    private double turnAngle; 
    private double hDistance; 
    private double heading; 
    
    
    //Initial constructor
    public Navigation(Odometer odo) {
        this.odo = odo;
    }

    public void run() { 
      //Loops through the waypoints sequentially
      for(int[] point : waypoints) {
        travelTo(point[0] * TILE_SIZE, point[1] * TILE_SIZE);
      }
    }
   
    public void travelTo( double x, double y) { 
        
        //Load current position from odometer
        pos = odo.getXYT(); 
        
        //Calculate distance to waypoint from current position
        dX = x - pos[0]; 
        dY = y - pos[1];
        
        //Calculate heading angle to next waypoint
        heading = Math.toDegrees(Math.atan2(dX, dY)); 
        turnAngle = heading - pos[2]; 
        
        //Rotate robot to calculated angle
        turnTo(turnAngle);
        
        //Calculate the min distance (hypotenuse) needed to reach next waypoint
        hDistance = Math.sqrt(dX * dX + dY * dY); 
        
        //Calls method that moves the robot to waypoint from forward distance
        toWaypoint(hDistance);  
    }
    
    //Computes small-angle turn required for robot to head to next waypoint, and turns the robot
    public void turnTo(double theta) {
      
        int wheelRotation; //
      
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

        wheelRotation = (int) ((TRACK * theta / 2) / (WHEEL_RADIUS)); //Calculates rotation needed based on Track and Wheel radius

        leftMotor.rotate(wheelRotation, true);
        rightMotor.rotate(-wheelRotation, false);
    }
    
    //Sets robot to move forward the distance to waypoint
    public void toWaypoint(double dis) {
        
      int wheelArc;
        
        leftMotor.setSpeed(FAST);
        rightMotor.setSpeed(FAST);
        
        wheelArc = (int)((dis * 180.0) / (WHEEL_RADIUS * Math.PI)); //Calculates arc angle needed for the distance to waypoint
        
        leftMotor.rotate(wheelArc, true);
        rightMotor.rotate(wheelArc, false);
    }
    
    //Checks robot motors, if moving, returns true boolean
    public boolean isNavigating() {
        return leftMotor.isMoving() && rightMotor.isMoving();  
    }

}

  