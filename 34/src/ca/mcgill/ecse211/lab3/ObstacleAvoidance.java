package ca.mcgill.ecse211.lab3;


import static ca.mcgill.ecse211.lab3.Resources.*;
import static java.lang.Math.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;


  public class ObstacleAvoidance extends Thread {
    
    /**
     * ObstacleAvoidance class for EV3
     * Allows robot to avoid obstacles on a set course
     */
    
    /*
     * Maps for navigation
     */
     
     private int[][] waypoints = {{2,0},{1,1},{1,2},{2,0}}; //MAP1
     //private int[][] waypoints = {{0, 2}, {1 ,1}, {2, 2}, {2, 1}, {1, 0}}; //MAP2
     //private int[][] waypoints = {{1, 1}, {0 ,2}, {2, 2}, {2, 1}, {1, 0}}; //MAP3
     //private int[][] waypoints = {{1, 0}, {2 ,1}, {2, 2}, {0, 2}, {1, 1}}; //MAP4
     //private int[][] waypoints = {{0, 1}, {1 ,2}, {1, 0}, {2, 1}, {2, 2}};   //MAP5
     //private int[][] waypoints = {{0, 2}, {2 ,2}, {2, 0}, {0, 0}}; //TESTMAP
    
    private double[] pos = new double[3]; //To store current position from odometer
    
    //Variables for travelTo()
    private double dX; 
    private double dY; 
    private double angleTurn; 
    private double hDistance; 
    private double heading; 
      
    private Odometer odo;
    
    //Variables for avoidance
    private double sensorAngle;
    private int sensorDist;
    private int wheelArc;
    private boolean detect;
    
    
    private SampleProvider us;
    private float[] usData;
    private boolean Navigate = true;
    private int distance;
    
    public ObstacleAvoidance(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
        EV3MediumRegulatedMotor mediumMotor,  SampleProvider us, float[] usData){
      
      
        this.odo = odo;}

    public void run() { 
      //Loops through the waypoints sequentially
      for(int[] point : waypoints) {
        try {
          travelTo(point[0] * TILE_SIZE, point[1] * TILE_SIZE);
        } catch (InterruptedException e) {
         
          e.printStackTrace();
        }
      }
    }
    
    
    public void travelTo( double x, double y) throws InterruptedException{ 
        //runSensor();
        
        //Load current position from odometer
        pos = odo.getXYT(); 
        
        //Calculate distance to waypoint from current position
        dX = x - pos[0]; 
        dY = y - pos[1];
        
        //Calculate heading angle to next waypoint
        heading = Math.toDegrees(Math.atan2(dX, dY)); 
        angleTurn = heading - pos[2]; 
        
        //Rotate robot to calculated angle
        turnTo(angleTurn);
        
        //Calculate the min distance (hypotenuse) needed to reach next waypoint
        hDistance = Math.sqrt(dX * dX + dY * dY); 
        
        //Calls method that moves the robot to waypoint from forward distance while avoiding obstacles
        avoidanceTravelTo(hDistance, x, y);  
    }
   
    //Sets robot to move forward the distance to waypoint
    //Handles obstacles avoidance
    public void avoidanceTravelTo(double dis, double x, double y) throws InterruptedException {
        detect = false;
        
        leftMotor.setSpeed(FAST);
        rightMotor.setSpeed(FAST);
        
        wheelArc = (int)((dis * 180.0) / (WHEEL_RADIUS * Math.PI)); //Calculates arc angle needed for the distance to waypoint
        
        leftMotor.rotate(wheelArc, true);
        rightMotor.rotate(wheelArc, true);
        
        //Obstacle avoidance
        while(isNavigating()) {
          sensorDist = UltrasonicPoller.getDistance();
          sensorAngle=UltrasonicPoller.getSensorAngle();
          
          if(sensorDist < 40) {
            mediumMotor.setSpeed(0);
            mediumMotor.stop();
          }
          if(sensorDist <= 22) {
            detect = true;
            if (sensorAngle < 0) {
              sensorAngle = 1;
              Sound.beep();
            }
            else {
              sensorAngle = -1;
              Sound.beepSequence();;
            }

            leftMotor.setSpeed(SLOW);
            rightMotor.setSpeed(SLOW);
            leftMotor.rotate(-270 * (int)sensorAngle,true); // increase left motor
            rightMotor.rotate(270  * (int)sensorAngle,false); // reverse right motor to pivot away
            
            leftMotor.rotate(800, true);
            rightMotor.rotate(800, false);
            
            leftMotor.rotate(270 * (int)sensorAngle, true);
            rightMotor.rotate(-270 * (int)sensorAngle, false);
            
            leftMotor.rotate(800, true);
            rightMotor.rotate(800, false);
            
            
  
            leftMotor.stop();
            rightMotor.stop();
            
          } 
        }
       if (detect == true) {                                                         
         travelTo(x,y);
       } 
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
        else {
        }
        try {
           Thread.sleep(500);
         } 
        catch (InterruptedException e) {  
         }

        wheelRotation = (int) ((180.0 * (TRACK * Math.PI * theta / 360)) / (WHEEL_RADIUS * Math.PI)); //Calculates rotation needed based on Track and Wheel radius

        leftMotor.rotate(wheelRotation, true);
        rightMotor.rotate(-wheelRotation, false);
    }
    
    //Checks robot motors, if moving, returns true boolean
    public boolean isNavigating() {
        return leftMotor.isMoving() && rightMotor.isMoving();  
    }
    
    public void runSensor() throws InterruptedException {
      sensorAngle = (int) Math.PI * 4 * mediumMotor.getTachoCount()/180;
      mediumMotor.setSpeed(100);
      mediumMotor.rotateTo((int)sensorAngle); 
      sensorAngle *=-1;
      
      mediumMotor.stop();
      
    }

}

  
