package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalization.ultrasonic;
import ca.mcgill.ecse211.project.Testing.Test;
import static ca.mcgill.ecse211.project.LightLocalization.*;
import static ca.mcgill.ecse211.project.Launcher.launchSequence;

/* 
 * ECSE-211, DPM
 * Fall 2019
 * Team 19: REEV3S
 */

/**
 * Main class for the final DPM project providing the functionalities for an EV3 robot
 * to localize, navigate through a closed circuit, avoid obstacles in its way, launch ping-pong balls
 * into a designated bin and return to its starting corner.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class Main {

  /**
   * Enumeration of program run modes to be uploaded on the EV3.
   *
   */
  enum Mode {
    TESTING, COMPETITION, DEMO
  };

  /**
   * Stores the program mode to be run
   */
  static Mode mode = Mode.COMPETITION;

  /**
   * Starting corner of the robot
   */
  public static int corner;

  /**
   * End double coordinates for the robot
   */
  private static double[] endPoint = {START[0], START[1]};

  public static void main(String[] args) throws InterruptedException {

    // State machine for switching between modes
    switch (mode) {

      case TESTING:
        Testing.startTest();

        if (Testing.test != Test.COMPETITION) {
          break;
        }

      case COMPETITION:
        startCompetition();
        break;

      case DEMO:
        startDemo();
        break;

      default:
        break;
    }
  }

  /**
   * Code sequence for the beta demo
   * 
   * @throws InterruptedException
   */
  private static void startDemo() throws InterruptedException {
    init();

    ultrasonic();

    initLsLocalization();
    light(corner);
    Sound.beep();

    toTunnelEntrance(START, TUNNEL_LL, TUNNEL_UR);

    navigateThroughTunnel(Navigation.tileExit);

    navigateToPoint(tileExit, launchPoint);

    int targetAngle = 100;
    turnTo(targetAngle);

    for (int i = 0; i < 3; i++) {
      Sound.beep();
    }

    launchSequence();
    Sound.beep();
  }

  /**
   * Code sequence for the final competition
   * 
   * @throws InterruptedException
   */
  private static void startCompetition() throws InterruptedException {
    // Start
    init();
    Main.sleepFor(50);

    // Ultrasonic localization
    ultrasonic();

    // Light localization
    initLsLocalization();
    light(corner);

    // 3 beeps after localization
    Main.sleepFor(50);
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Main.sleepFor(50);

    // Travel to tunnel entrance with odometry correction
    toTunnelEntrance(START, TUNNEL_LL, TUNNEL_UR);

    // Localize before entering tunnel
    localizeBeforeTunnel();

    // Navigate through the tunnel with the launch arm down
    navigateThroughTunnel(tileExit);

    // Localize after exiting the tunnel
    localizeAfterTunnel();

    // Set launchPoint
    launchPoint[0] = odometer.getX() / TILE_SIZE;
    launchPoint[1] = odometer.getY() / TILE_SIZE;

    // 3 beeps at launch point before launch sequence
    Main.sleepFor(50);
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Main.sleepFor(50);

    // Launch all ping-pong balls
    launchSequence(launchPoint, TARGET);

    // Localize after launch
    localizeLaunch();

    // Navigate back to tunnel exit after localizing around launch point
    Main.sleepFor(50);
    launchPoint[0] = odometer.getX() / TILE_SIZE;
    launchPoint[1] = odometer.getY() / TILE_SIZE;
    navigateToPoint(launchPoint, tileExit);

    // Navigate back through the tunnel to team zone
    navigateBackThroughTunnel(tileEntrance);

    // Navigate to the starting corner
    navigateToPoint(tileEntrance, endPoint);

    // 5 beeps after returning to starting corner
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();

  }

  /**
   * Method to initialize all threads and wait for reception of WIFI parameters
   */
  private static void init() {
    new Thread(odometer).start();
    new Thread(usPoller).start();
    new Thread(lsPoller).start();

    setupMotors();

    receiveWifiParameters(RECEIVE_WIFI_PARAMS);
  }

  /**
   * Method to initialize light localization at the beginning
   */
  private static void initLsLocalization() {
    LightPoller.startLSPolling();
    lsPoller.initializeLightSensor(RIGHT_LIGHT_SENSOR);
    lsPoller.initializeLightSensor(LEFT_LIGHT_SENSOR);
  }

  /**
   * Set parameters from Wifi reception
   * 
   * @param receiveParams Receive wifi parameters if true
   */
  private static void receiveWifiParameters(boolean receive_wifi_params) {
    if (receive_wifi_params) {

      if (TEAM_NUMBER == redTeam) {
        ZONE_LL[0] = red.ll.x;
        ZONE_LL[1] = red.ll.y;
        ZONE_UR[0] = red.ur.x;
        ZONE_UR[1] = red.ur.y;

        TUNNEL_LL[0] = (int) tnr.ll.x;
        TUNNEL_LL[1] = (int) tnr.ll.y;
        TUNNEL_UR[0] = (int) tnr.ur.x;
        TUNNEL_UR[1] = (int) tnr.ur.y;

        ISLAND_LL[0] = (int) Math.ceil(island.ll.x);
        ISLAND_LL[1] = (int) Math.ceil(island.ll.y);
        ISLAND_UR[0] = (int) Math.floor(island.ur.x);
        ISLAND_UR[1] = (int) Math.floor(island.ur.y);

        TARGET[0] = (int) redBin.x;
        TARGET[1] = (int) redBin.y;

        corner = redCorner;
      } else {
        ZONE_LL[0] = green.ll.x;
        ZONE_LL[1] = green.ll.y;
        ZONE_UR[0] = green.ur.x;
        ZONE_UR[1] = green.ur.y;

        TUNNEL_LL[0] = (int) tng.ll.x;
        TUNNEL_LL[1] = (int) tng.ll.y;
        TUNNEL_UR[0] = (int) tng.ur.x;
        TUNNEL_UR[1] = (int) tng.ur.y;

        ISLAND_LL[0] = (int) Math.ceil(island.ll.x);
        ISLAND_LL[1] = (int) Math.ceil(island.ll.y);
        ISLAND_UR[0] = (int) Math.floor(island.ur.x);
        ISLAND_UR[1] = (int) Math.floor(island.ur.y);

        TARGET[0] = (int) greenBin.x;
        TARGET[1] = (int) greenBin.y;

        corner = greenCorner;
      }

      if (corner == 0) {
        START[0] = 1;
        START[1] = 1;
      } else if (corner == 1) {
        START[0] = 14;
        START[1] = 1;
      } else if (corner == 2) {
        START[0] = 14;
        START[1] = 8;
      } else {
        START[0] = 1;
        START[1] = 8;
      }
    }
  }

  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

}
