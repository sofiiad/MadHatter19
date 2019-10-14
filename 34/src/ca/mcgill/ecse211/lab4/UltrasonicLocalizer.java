package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.lab4.Resources.*;
public class UltrasonicLocalizer {
  
//RESOURCES

public static final float MAX_DIST = 100;
public static final double WALL_DIST = 40;

public static final double WALL_DIST1 = 120;
public static final double INTERCEPT_DIST = 20;
public static final double MARGIN_DIST = 3;
public static final double OFFSET_ANGLE = 180;
public static final double DIAGONAL_ANGLE = 45;
public static final double LOCALMAX_ANGLE = 225;

Odometer odo;
private Navigation nav;
private static SampleProvider usDistance;
private static float[] usData = new float[usSensor.sampleSize()];
private boolean choseRising;

private static float filteredDist;
private boolean inNoiseMargin = false;
private static double angleA = 0;
private static double angleB = 0;
private double prevAngle;
private double nextAngle;
private double dTheta;


  public UltrasonicLocalizer(Odometer odo, Navigation nav, SampleProvider usDistance, float[] usData, boolean choseRising) {
      this.odo = odo;
      this.nav = nav;
      this.usDistance = usDistance;
      choseRising = this.choseRising;
      
  }
  
  public void USRoutine(boolean choseRising) {
    if(choseRising) {
      RisingEdgeRoutine();
    }
    else if(!choseRising){
      FallingEdgeRoutine();
    }
  }
      //falling edge 
  public void FallingEdgeRoutine() {  
      //turn clockwise
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.forward();
      rightMotor.backward();
      
        while(true) {
          if(getFilteredDist() >= WALL_DIST) {
            break;
          }
        }
      
      //once no more wall
      //Get first angle
        while(true) {     //finds wall
          if(!inNoiseMargin && getFilteredDist() < INTERCEPT_DIST + MARGIN_DIST) {
            inNoiseMargin = true;
            prevAngle = odometer.getXYT()[2];
            Sound.beep();
           }
          else if(inNoiseMargin && getFilteredDist() < INTERCEPT_DIST - MARGIN_DIST){
            Sound.beepSequence();
            inNoiseMargin = false;
            nextAngle = odometer.getXYT()[2];

            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            break;
          }
         }
     
      angleA = (prevAngle + nextAngle)/2;
      
      //turn counter clockwise
      
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.backward();
      rightMotor.forward();
      while(true) {
        if(getFilteredDist() > WALL_DIST) {
          Sound.beepSequenceUp();
          break;
        }
      }
      
      //once no more wall
      //Get second angle
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.backward();
      rightMotor.forward();
      while(true) {
        if(!inNoiseMargin && getFilteredDist() < INTERCEPT_DIST + MARGIN_DIST) {     //finds wall
          inNoiseMargin = true;
          prevAngle = odometer.getXYT()[2];
         }
        else if(inNoiseMargin && getFilteredDist() < INTERCEPT_DIST - MARGIN_DIST){
          inNoiseMargin = false;
          nextAngle = odometer.getXYT()[2];
          Sound.beepSequenceUp();
          break;
       }
      }
      
      
      angleB = (prevAngle + nextAngle)/2;    
      
      if(angleA > angleB) {
        dTheta = angleA - (angleA + angleB)/2 - 25;
        Sound.beep();
      }
      else {
        dTheta = angleB - (angleA + angleB)/2 + 10;
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
      }
      
//      
//      if(dTheta <=180) {
//        dTheta += 360;
//      }
//      else if(dTheta > 180) {
//        dTheta -= 360;
//      }
      
      //double zero = angleB - dTheta;
      
      double angleDist = Math.PI * TRACK * dTheta /360;
      int converted = (int) ((180 * angleDist) / (Math.PI * WHEEL_RADIUS));
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.rotate(converted, true);
      rightMotor.rotate(-converted, false);
      
      
      odometer.setXYT(0, 0, 0);
      
  }
  
    public void RisingEdgeRoutine() {
    //turn clockwise
      leftMotor.setSpeed(SLOW-50);
      rightMotor.setSpeed(SLOW-50);
      leftMotor.backward();
      rightMotor.forward();
      while(true) {
        if(getFilteredDist() < 30) {//WALL_DIST1) {
          break;
        }
      }
    
    //once wall found
    //Get first angle
      while(true) {
        if(!inNoiseMargin && getFilteredDist() > 60) { //INTERCEPT_DIST - MARGIN_DIST) {     //finds wall
          inNoiseMargin = true;
          prevAngle = odometer.getXYT()[2];
          Sound.beep();
         }
        else if(inNoiseMargin && getFilteredDist() > 65) { //INTERCEPT_DIST + MARGIN_DIST){
          Sound.beepSequence();
          inNoiseMargin = false;
          nextAngle = odometer.getXYT()[2];
          
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          break;
        }
      } 
  
     
      angleA = (prevAngle + nextAngle)/2;
      
      //turn counter clockwise
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.forward();
      rightMotor.backward();
      while(true) {
        if(getFilteredDist() < WALL_DIST) {
          Sound.beepSequenceUp();
          break;
        }
      }
      
      //once wall found
      //Get second angle
      while(true) {
        if(!inNoiseMargin && getFilteredDist() > 60) {//INTERCEPT_DIST - MARGIN_DIST) {     //finds wall
          inNoiseMargin = true;
          prevAngle = odometer.getXYT()[2];
         }
        else if(inNoiseMargin && getFilteredDist() > 65) {//INTERCEPT_DIST + MARGIN_DIST){
          Sound.beepSequenceUp();
          inNoiseMargin = false;
          nextAngle = odometer.getXYT()[2];
          break;
        }
       }
      
      angleB = (prevAngle + nextAngle)/2;    
      
      if(angleA > angleB) {
        dTheta = angleA - (angleA + angleB)/2 - 45;
      }
      else if(angleA < angleB) {
        dTheta = angleB - (angleA + angleB)/2 - 45;
      }
      
      double angleDist = Math.PI * TRACK * dTheta /360;
      int converted = (int) ((180 * angleDist) / (Math.PI * WHEEL_RADIUS));
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.rotate(converted, true);
      rightMotor.rotate(-converted, false);
      
      odometer.setXYT(0, 0, 0);
    }
    
    public static float getFilteredDist() {
      usDistance.fetchSample(usData, 0);
      filteredDist = usData[0] * 100;
      if(filteredDist > MAX_DIST) {
        return MAX_DIST;
      }
      return filteredDist;
    }
    
    public static double getA() {
      double A = angleA;
      return A;
    }
    
    public static double getB() {
      double B = angleB;
      return B;
    }
   

}
