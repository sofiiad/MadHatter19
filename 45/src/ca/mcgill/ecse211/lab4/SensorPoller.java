package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Sound;

/**
 * State machine which polls different sensors for the right values ie distance and light intensity
 * @author KarlKoerich
 * @author Alex Choi
 */

public class SensorPoller extends Thread {
  public static float[] usData = new float[US_SENSOR.sampleSize()];
  public static float[] lightData = new float[LIGHT_SENSOR.sampleSize()];
  public static int distance = 0;
  public static int intensity = 0;
  long snapshot;
  long currentTime;
  volatile long wait_period = US_WAIT;
  
  public void run() {
    while (true) {
      snapshot = System.currentTimeMillis();
      switch(LOCAL_STATUS) {
        case ULTRASONIC:
          US_SENSOR.getDistanceMode().fetchSample(usData, 0);
          distance = (int) (usData[0] * 100.0);
          if (distance > 300) distance = 300;
          break;
          
        case LIGHT:
          distance = 0;
          wait_period = LS_WAIT;
          LIGHT_SENSOR.getRedMode().fetchSample(lightData, 0);
          intensity = (int) (lightData[0] * 1000);
          break;
          
        default:
          LCD.drawString("ERROR!!!", 0, 0);
          Sound.beep();
          System.exit(1);

      }
      
      currentTime = System.currentTimeMillis();
      if (currentTime - snapshot < wait_period) {
        Main.sleepFor(wait_period - (currentTime - snapshot));
      }
    } 
  }
}
