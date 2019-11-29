package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.LEFT_LIGHT_SENSOR;
import static ca.mcgill.ecse211.project.Resources.RIGHT_LIGHT_SENSOR;
import static ca.mcgill.ecse211.project.Resources.LS_WAIT;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * LightPoller thread. Class offers static methods to control the thread and get the detection of lines by each of two
 * light sensors as booleans.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class LightPoller implements Runnable {

  /**
   * Thread sleep
   */
  public static boolean sleep = false;

  /**
   * Proportion of environment light intensity considered to represent black line reading
   */
  private static final double LIGHT_TRESHOLD_CONSTANT = 0.5;

  /**
   * Light intensity of the environment used by the left light sensor to detect black lines
   */
  public static int leftEnvIntensity = 0;

  /**
   * Light intensity of the environment used by the right light sensor to detect black lines
   */
  public static int rightEnvIntensity = 0;

  /**
   * Array used to store left light sensor data
   */
  private static float[] leftLSData = new float[LEFT_LIGHT_SENSOR.sampleSize()];

  /**
   * Array used to store right light sensor data
   */
  private static float[] rightLSData = new float[RIGHT_LIGHT_SENSOR.sampleSize()];

  /**
   * Stores sleep time of thread in ns
   */
  private static int ns = 200;

  ///// CLASS VARIABLES USED TO STORE SENSOR DATA
  ///// /////////////////////////////////////////////////////////

  /**
   * Flag for line detected by left light sensor
   */
  private static volatile boolean leftLS_lineDetected = false;

  /**
   * Flag for line detected by right light sensor
   */
  private static volatile boolean rightLS_lineDetected = false;

  ///// FLAGS USED FOR SENSOR POLLING
  ///// /////////////////////////////////////////////////////////////////////

  /**
   * True if we are polling the left light sensor
   */
  public static boolean pollingLeftLS = false;

  /**
   * True if we are polling the right light sensor
   */
  public static boolean pollingRightLS = false;

  /**
   * This thread manages polling of all light sensors
   */
  public void run() {

    // Initialize corresponding light sensor(s)

    long snapshot, currentTime, dT;

    while (true) {

      snapshot = System.currentTimeMillis();

      if (pollingRightLS) {
        pollLightSensor(RIGHT_LIGHT_SENSOR);

      }

      if (pollingLeftLS) {
        pollLightSensor(LEFT_LIGHT_SENSOR);

      }

      // Wait until next polling
      currentTime = System.currentTimeMillis();
      dT = currentTime - snapshot;
      if (dT < LS_WAIT) {
        Main.sleepFor(LS_WAIT - dT);
      }
    }
  }

  ///// LIGHT SENSOR PRIVATE METHODS
  ///// /////////////////////////////////////////////////////////////////////

  /**
   * 
   * Poll light sensor
   * 
   * @param lightSensor
   */
  private synchronized void pollLightSensor(EV3ColorSensor lightSensor) {

    if (sleep) {
      try {
        Thread.sleep(ns);
        sleep = false;
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }

    } else {
      // Poll left
      if (lightSensor.equals(LEFT_LIGHT_SENSOR)) {

        LEFT_LIGHT_SENSOR.getRedMode().fetchSample(leftLSData, 0);

        if (leftLSData[0] * 1000 < LIGHT_TRESHOLD_CONSTANT * leftEnvIntensity) {
          leftLS_lineDetected = true;
        } else {
          leftLS_lineDetected = false;
        }
      }
      // Poll right
      if (lightSensor.equals(RIGHT_LIGHT_SENSOR)) {

        RIGHT_LIGHT_SENSOR.getRedMode().fetchSample(rightLSData, 0);

        if (rightLSData[0] * 1000 < LIGHT_TRESHOLD_CONSTANT * rightEnvIntensity) {
          rightLS_lineDetected = true;
        } else {
          rightLS_lineDetected = false;
        }
      }

    }

  }

  /**
   * Set the environment light intensity used for comparison in detecting black lines
   */
  public void initializeLightSensor(EV3ColorSensor lightSensor) {
    if (lightSensor.equals(LEFT_LIGHT_SENSOR)) {
      lightSensor.getRedMode().fetchSample(leftLSData, 0);
      leftEnvIntensity = (int) (leftLSData[0] * 1000);
    } else if (lightSensor.equals(RIGHT_LIGHT_SENSOR)) {
      lightSensor.getRedMode().fetchSample(rightLSData, 0);
      rightEnvIntensity = (int) (rightLSData[0] * 1000);
    }
  }

  ///// PUBLIC METHODS
  ///// ///////////////////////////////////////////////////////////////////////////////////

  /**
   * Start polling specific light sensor
   * 
   * @param lightSensor
   */
  public static void startPolling(EV3ColorSensor lightSensor) {

    if (lightSensor.equals(LEFT_LIGHT_SENSOR)) {
      pollingLeftLS = true;
    } else if (lightSensor.equals(RIGHT_LIGHT_SENSOR)) {
      pollingRightLS = true;
    }

  }

  /**
   * Stop polling specific light sensor
   * 
   * @param lightSensor
   */
  public static void stopPolling(EV3ColorSensor lightSensor) {

    if (lightSensor.equals(LEFT_LIGHT_SENSOR)) {
      pollingLeftLS = false;
    } else if (lightSensor.equals(RIGHT_LIGHT_SENSOR)) {
      pollingRightLS = false;
    }

  }

  /**
   * Start polling all light sensors
   */
  public static void startLSPolling() {
    startPolling(LEFT_LIGHT_SENSOR);
    startPolling(RIGHT_LIGHT_SENSOR);
  }

  /**
   * Stop polling all light sensors
   */
  public static void stopLSPolling() {
    stopPolling(LEFT_LIGHT_SENSOR);
    stopPolling(RIGHT_LIGHT_SENSOR);
  }

  /**
   * Returns whether the left LS has detected a line and resets flag to false
   * 
   * @return boolean lineDetected
   */
  public static synchronized boolean leftLS_isLineDetected() {
    boolean lineDetected = leftLS_lineDetected;
    leftLS_lineDetected = false;
    return lineDetected;
  }

  /**
   * Returns whether the right LS has detected a line and resets flag to false
   * 
   * @return boolean lineDetected
   */
  public static synchronized boolean rightLS_isLineDetected() {
    boolean lineDetected = rightLS_lineDetected;
    rightLS_lineDetected = false;
    return lineDetected;
  }

  /**
   * Method to sleep the LightPoller thread
   */
  public static synchronized void sleep() {
    sleep = true;
    ns = 200;
  }

  /**
   * Method to sleep the LightPoller thread for a specified time
   * 
   * @param sleepTime duration of sleep
   */
  public synchronized static void sleepFor(int sleepTime) {
    sleep = true;
    ns = sleepTime;
  }
}
