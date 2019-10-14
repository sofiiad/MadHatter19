package ca.mcgill.ecse211.lab4;
import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer{
  
  public static final double LIGHT_MARGIN = 10;
  
  private Odometer odo;
  private Navigation nav;
  private EV3ColorSensor colorSensor;
  private static final long CORRECTION_PERIOD = 10;
  private long correctionStart, correctionEnd;
  
  private static boolean isTravellingY;
  
  //colorSensor variables
  private float sensorTrigger;
  private float[] sensorValue;
  
  public static double[] tile = {30.48, 30.48};

  public LightLocalizer(Odometer odo, Navigation nav, EV3ColorSensor colorSensor) {
    this.odo = odo;
    this.nav = nav;
    this.colorSensor = colorSensor;
  }
  
  public void run() {
    correctionStart = System.currentTimeMillis();
    colorSensor.setCurrentMode("Red");
    sensorValue = new float[1];
    sensorTrigger = (float) 0.267;
    
    int lineOffset = (int)((OFFSET * 180.0) / (WHEEL_RADIUS * Math.PI)); //Calculates arc angle needed for the distance to waypoint
    
    

    isTravellingY = true;
    //sensorData Get sensor data
    
    leftMotor.setSpeed(SLOW);
    rightMotor.setSpeed(SLOW);
    leftMotor.forward();
    rightMotor.forward();
    while(isTravellingY) {
      colorSensor.fetchSample(sensorValue, 0);
     
      if(sensorValue[0] < sensorTrigger) { //Check for black line (low return value)
        Sound.beep();
        leftMotor.rotate(lineOffset, true);
        rightMotor.rotate(lineOffset, false);
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        odo.setY(tile[0]);
        isTravellingY = false;
      }
    }
      //turn robot 90 degrees
      nav.turnTo(91);
      
      leftMotor.setSpeed(SLOW);
      rightMotor.setSpeed(SLOW);
      leftMotor.forward();
      rightMotor.forward();
      while(!isTravellingY) {
        
        colorSensor.fetchSample(sensorValue, 0);
        if(sensorValue[0] < sensorTrigger) { //Check for black line (low return value)
          Sound.beep();
          leftMotor.rotate(lineOffset, true);
          rightMotor.rotate(lineOffset, false);
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          odo.setX(tile[1]);
          isTravellingY = true;
        }
    }
  }
  
  
}
