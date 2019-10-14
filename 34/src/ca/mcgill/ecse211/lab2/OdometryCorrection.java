package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private long correctionStart, correctionEnd;
  
  //Position-related variables
  int lineCount;
  private double[] position;
  private double cosTheta;
  private double sinTheta;
  
  //colorSensor variables
  private float sensorTrigger;
  private float[] sensorValue;
  
  /*
   * Here is where the odometer correction code should be run.
   */
  public void run() {
    
	colorSensor.setCurrentMode("Red");
    sensorValue = new float[1];
    sensorTrigger = (float) 0.265;
    lineCount = 0;
   
    while (true) {
      correctionStart = System.currentTimeMillis();
      colorSensor.fetchSample(sensorValue, 0);
      
      position = odometer.getXYT();
      cosTheta = Math.cos(Math.toRadians(position[2]));
      sinTheta = Math.sin(Math.toRadians(position[2]));
      
      //Trigger correction
      if(sensorValue[0] < sensorTrigger) { //Check for black line (low return value)
        lineCount += 1;
        Sound.beep();
        
        if(Math.abs(cosTheta) > 0.5) { //Check for robot moving up or down the Y-axis        
          
          //Calculate new (accurate) robot Y-coord
          if(cosTheta < 0) { //Moving down
        	position[1] = (4 - lineCount) * TILE_SIZE + OFFSETY; //Starting from last line, down [3,2,1]
          }
          
          else { //Moving up
        	position[1] = lineCount * TILE_SIZE - OFFSETY; //Starting from first line, up [1,2,3] 
          }
        }
        
        else if(Math.abs(sinTheta) > 0.5) { //Check for robot moving up or down the X-axis 
          
          //Calculate new (accurate) robot X-coord
          if(sinTheta < 0) { //Moving down 
        	position[0] = (4 - lineCount) * TILE_SIZE + OFFSETX; //Starting from last line, down [3,2,1]
          }
          
          else { //Moving up
        	position[0] = lineCount * TILE_SIZE - OFFSETX; //Starting from first line, up [1,2,3]
          }
        }
        
        //Update odometer with new calculated (and more accurate) values
        odometer.setXYT(position[0], position[1], position[2]);
        
        lineCount = lineCount % 3; //Reset lineCount after every 3 lines
        
      }


      // TODO Update odometer with new calculated (and more accurate) values, eg:
      //odometer.setXYT(0.3, 19.23, 5.0);

      // this ensures the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }
  
}
