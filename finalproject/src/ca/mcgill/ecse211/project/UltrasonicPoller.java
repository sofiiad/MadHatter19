package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.US_SENSOR_FRONT;
import static ca.mcgill.ecse211.project.Resources.US_WAIT;
import static ca.mcgill.ecse211.project.Resources.MAXIMUM_DISTANCE;
import static ca.mcgill.ecse211.project.Resources.US_MEDIAN_FILTER;
import java.util.Arrays;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * UltrasonicPoller thread. Class offers static methods to control the thread and get the ultrasonic sensor's median
 * filtered distance readings.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class UltrasonicPoller implements Runnable {

  /**
   * Boolean to control sleeping the ultrasonic poller.
   */
  public static boolean sleep = false;

  /**
   * Array used to store last ultrasonic readings
   */
  private static int[] usReadings = new int[US_MEDIAN_FILTER];

  /**
   * Keeps track of number of ultrasonic readings
   */
  private static int usReadingsCounter;

  /**
   * Array used to store ultrasonic sensor data
   */
  private static float[] usData = new float[US_SENSOR_FRONT.sampleSize()];

  /**
   * Stores sleep time of UltrasonicSensor in ns
   */
  private static int ns = 15000;

  /**
   * Distance read by the front ultrasonic sensor
   */
  private static int usPollerDistance;

  ///// FLAG USED FOR SENSOR POLLING
  ///// /////////////////////////////////////////////////////////////////////

  /**
   * True if we are polling the front ultrasonic sensor
   */
  public static boolean pollingUS_Front = false;

  ///// POLLING THREAD
  ///// ///////////////////////////////////////////////////////////////////////////////////

  /**
   * This thread manages polling of US sensor
   */
  public void run() {

    long snapshot, currentTime, dT;

    while (true) {

      if (sleep) {
        try {
          Thread.sleep(ns);
          sleep = false;
        } catch (InterruptedException ex) {
          Thread.currentThread().interrupt();
        }
      } else {
        snapshot = System.currentTimeMillis();

        if (pollingUS_Front) {
          pollUltrasonicSensor(US_SENSOR_FRONT);
        }
        // Wait until next polling
        currentTime = System.currentTimeMillis();
        dT = currentTime - snapshot;
        if (dT < US_WAIT) {
          Main.sleepFor(US_WAIT - dT);
        }
      }
    }
  }

  ///// ULTRASONIC SENSOR PRIVATE METHODS
  ///// ////////////////////////////////////////////////////////////////

  /**
   * Poll the specified ultrasonic sensor
   * 
   * @param usSensor Ultrasonic sensor for poll
   */
  private void pollUltrasonicSensor(EV3UltrasonicSensor usSensor) {

    int distance;

    if (usSensor.equals(US_SENSOR_FRONT)) {

      // Get ultrasonic sensor data
      US_SENSOR_FRONT.getDistanceMode().fetchSample(usData, 0);

      // Convert distance to cm
      distance = (int) (usData[0] * 100.0);

      if (distance > MAXIMUM_DISTANCE) {
        distance = MAXIMUM_DISTANCE;
      }

      for (int i = 0; i < US_MEDIAN_FILTER; i++) {
        usReadings[usReadingsCounter++ % US_MEDIAN_FILTER] = distance;
      }

      usPollerDistance = calculateMedian(usReadings.clone(), US_MEDIAN_FILTER);
    }
  }

  /**
   * Calculate the median of the array
   * 
   * @param array
   * @param length
   * @return median
   */
  private int calculateMedian(int[] array, int length) {

    // Sort array
    Arrays.sort(array);

    // Return array median
    return array[length / 2];
  }

  ///// PUBLIC METHODS
  ///// ///////////////////////////////////////////////////////////////////////////////////

  /**
   * Start polling ultrasonic sensor
   * 
   * @param usSensor Ultrasonic sensor to be started
   */
  public static void startPolling(EV3UltrasonicSensor usSensor) {

    if (usSensor.equals(US_SENSOR_FRONT)) {
      pollingUS_Front = true;

      for (int i = 0; i < US_MEDIAN_FILTER; i++) {
        usReadings[i] = MAXIMUM_DISTANCE;
      }

      usReadingsCounter = 0;
    }
  }

  /**
   * Stop polling ultrasonic sensor
   * 
   * @param usSensor Ultrasonic sensor to be stopped
   */
  public static void stopPolling(EV3UltrasonicSensor usSensor) {
    if (usSensor.equals(US_SENSOR_FRONT)) {
      pollingUS_Front = false;
    }
  }

  /**
   * Gets the filtered distance detected by the front ultrasonic sensor
   * 
   * @return usPollerDistance
   */
  public static int getUsPollerDistance() {
    return usPollerDistance;
  }

  /**
   * Method to sleep the UltrasonicPoller thread
   */
  public static synchronized void sleep() {
    sleep = true;
    ns = 600;
  }

  /**
   * Method to sleep the UltrasonicPoller thread for a specified time
   * 
   * @param sleepTime duration of sleep in ns
   */
  public static synchronized void sleepFor(int sleepTime) {
    sleep = true;
    ns = sleepTime;
  }

}
