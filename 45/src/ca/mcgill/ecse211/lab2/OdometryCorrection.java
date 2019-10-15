package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  static float[] sampleColor = new float[colorSensor.sampleSize()];
  double thetaUncertainty = 10;
  int prevData = 430;
  
  int prev_direction = 12;
  //boolean first_line_after_turn = true;
  
  public int direction (double theta) {
    
    if (theta >= 0 && theta <= (thetaUncertainty) || theta >= (360-thetaUncertainty) && theta <= 360) {
      return 12;
    }
    
    else if (theta <= (270 + thetaUncertainty) && theta >= (270 - thetaUncertainty)) {
      return 3;
    }
    
    else if (theta <= (90 + thetaUncertainty) && theta >= (90 - thetaUncertainty)) {
      return 9;
    }
    
    else if (theta <= (180 + thetaUncertainty) && theta >= (180 - thetaUncertainty)) {
      return 6;
    }
    
    else return 0;  
  }
  

  /*
   * Here is where the odometer correction code should be run.
   */
  public void run() {
    long correctionStart, correctionEnd;
    
    
    while (true) {
      correctionStart = System.currentTimeMillis();
      colorSensor.getRedMode().fetchSample(sampleColor, 0);
      int data = (int) (sampleColor[0] * 1000);

      // TODO Trigger correction (When do I have information to correct?)
      if (data < 300) { // //Safe value from data readings
        
        double[] positions = odometer.getXYT();
        double y = positions[1];
        double x = positions[0];
        
        int current_direction = direction(positions[2]);
        
        if (current_direction == 12) {
          Sound.beep();
          double correctionY = (TILE_SIZE) - (y % TILE_SIZE);
          //System.out.println("Corrected 12: " + correctionY + " y % TILE_SIZE: " + (y % TILE_SIZE));
          odometer.setY(y + correctionY - CS_DISTANCE);
          
        } else if (direction(positions[2]) == 3){
          Sound.beep();
          double correctionX = (TILE_SIZE) - (x % TILE_SIZE);
          //System.out.println("Corrected 3: " + correctionX + " x % TILE_SIZE: " + (x % TILE_SIZE));
          odometer.setX(x + correctionX - CS_DISTANCE);
          
        } else if (direction(positions[2]) == 6) {
          Sound.beep();
          double correctionY = (y % TILE_SIZE);
          //System.out.println("Corrected 6: " + correctionY);// + " y % TILE_SIZE: " + (y % TILE_SIZE));
          odometer.setY(y - correctionY + CS_DISTANCE);
          
        } else if (direction(positions[2]) == 9) {
          Sound.beep();
          double correctionX = (x % TILE_SIZE);
          //System.out.println("Corrected 9: " + correctionX); // + " x % TILE_SIZE: " + (x % TILE_SIZE));
          odometer.setX(x - correctionX + CS_DISTANCE);
          
        }
        
        
      }
      
      
      // prevData = data;
      // this ensures the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }
  
}
