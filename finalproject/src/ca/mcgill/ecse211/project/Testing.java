package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.LightPoller.*;
import ca.mcgill.ecse211.project.Main.Mode;
import static ca.mcgill.ecse211.project.Launcher.*;

/**
 * Class that offers private static methods for both contained unit and integration tests.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class Testing {

  /**
   * Enumerator of test options for quick switching
   *
   */
  enum Test {
    TRACK, WHEELBASE, LAUNCH, LOWER, LS_LOCALIZATION, US_POLLER, LS_POLLER, US_LOCALIZATION, 
    ODO_CORRECTION, BEFORE_TUNNEL_LOCALIZATION, AFTER_TUNNEL_LOCALIZATION, COMPETITION
  };

  static Test test = Test.COMPETITION;

  public static void startTest() throws InterruptedException {

    switch (test) {

      case TRACK:
        trackTest();
        break;

      case WHEELBASE:
        wheelBaseTest();
        break;

      case LAUNCH:
        launchTest();
        break;

      case LOWER:
        lowerLaunchTest();
        break;

      case US_POLLER:
        ultrasonicPollerTest();
        break;

      case LS_POLLER:
        lightPollerTest();
        break;

      case US_LOCALIZATION:
        ultrasonicLocalizationTest();
        break;

      case LS_LOCALIZATION:
        lightLocalizationTest();
        break;

      case BEFORE_TUNNEL_LOCALIZATION:
        beforeTunnelLocalizationTest();
        break;

      case AFTER_TUNNEL_LOCALIZATION:
        afterTunnelLocalizationTest();
        break;

      case ODO_CORRECTION:
        odometryCorrectionTest();
        break;

      case COMPETITION:
        // Testing without WIFI params
        // Some test maps are left commented out for convenience
        ZONE_LL[0] = 0;// 0;//0
        ZONE_LL[1] = 5;// 0;//0
        ZONE_UR[0] = 4;// 7;//7
        ZONE_UR[1] = 9;// 4;//4

        TUNNEL_LL[0] = 4;// 4;//3;//0;//4
        TUNNEL_LL[1] = 4;// 7;//1;//1;//4
        TUNNEL_UR[0] = 5;// 6;//5;//1;//5
        TUNNEL_UR[1] = 6;// 8;//0;//6;//6

        ISLAND_LL[0] = 6;// 0;//2
        ISLAND_LL[1] = 5;// 6;//5
        ISLAND_UR[0] = 15;// 8;//15
        ISLAND_UR[1] = 9;// 9;//9

        Main.corner = 3;// 0;//0

        TARGET[0] = -5;// 5;//4;//-1
        TARGET[1] = 7;// -2;//12;//-1

        START[0] = 1;
        START[1] = 8;

        Main.mode = Mode.COMPETITION;
        break;

      default:
        break;
    }

  }

  /**
   * Method for testing the LightPoller
   */
  private static void lightPollerTest() {
    new Thread(odometer).start();
    new Thread(lsPoller).start();

    lsPoller.initializeLightSensor(LEFT_LIGHT_SENSOR);
    lsPoller.initializeLightSensor(RIGHT_LIGHT_SENSOR);

    startPolling(LEFT_LIGHT_SENSOR);
    startPolling(RIGHT_LIGHT_SENSOR);

  }

  /**
   * Method for testing the UltrasonicPoller
   */
  private static void ultrasonicPollerTest() {
    new Thread(usPoller).start();

    Navigation.moveForward(500);
  }

  /**
   * Method for testing the robot's wheel radius
   */
  private static void trackTest() {
    new Thread(odometer).start();

    Navigation.moveForward(TILE_SIZE * 5);
    System.out.print("\nx: " + odometer.getX() + "\ny: " + odometer.getY() + "\nt: " + odometer.getT());
  }

  /**
   * Method for testing the robot's track/wheelbase
   */
  private static void wheelBaseTest() {
    new Thread(odometer).start();

    Navigation.turnTo(45);
    System.out.print("\nx: " + odometer.getX() + "\ny: " + odometer.getY() + "\nt: " + odometer.getT());
  }

  /**
   * Method for testing the launch sequence from the Launcher class
   */
  private static void launchTest() {
    odometer.setTheta(0);
    double[] current = {1, 1};
    int[] target = {8, 10};

    launchSequence(current, target);
  }

  /**
   * Method for testing the lowering of the launch arm through a tunnel
   */
  private static void lowerLaunchTest() {
    new Thread(odometer).start();

    Navigation.tileExit[0] = 1.5;
    Navigation.tileExit[1] = 2.5;

    Navigation.tileEntrance[0] = 1.5;
    Navigation.tileEntrance[1] = 4.5;
    Navigation.isHorizontalTunnel = false;

    odometer.setXYT(Navigation.tileEntrance[0] * 30.48, Navigation.tileEntrance[1] * 30.48, 0);

    Main.sleepFor(100);
    Navigation.navigateThroughTunnel(Navigation.tileExit);
  }

  /**
   * Method for testing light localization and its accuracy
   */
  private static void lightLocalizationTest() {
    new Thread(odometer).start();
    new Thread(lsPoller).start();

    lsPoller.initializeLightSensor(LEFT_LIGHT_SENSOR);
    lsPoller.initializeLightSensor(RIGHT_LIGHT_SENSOR);

    LightPoller.startLSPolling();

    Navigation.setupMotors();
    LightLocalization.light(0); // Passed argument is a corner

    System.out.print("\nx: " + odometer.getX() + "\ny: " + odometer.getY() + "\nt: " + odometer.getT());
  }

  /**
   * Method for testing ultrasonic localization and its reliability
   */
  private static void ultrasonicLocalizationTest() {
    new Thread(odometer).start();
    new Thread(usPoller).start();

    UltrasonicPoller.startPolling(US_SENSOR_FRONT);

    Navigation.setupMotors();
    UltrasonicLocalization.ultrasonic();

    System.out.print("\nx: " + odometer.getX() + "\ny: " + odometer.getY() + "\nt: " + odometer.getT());
  }

  /**
   * Method for testing odometry correction from a starting point to the tunnel entrance
   */
  private static void odometryCorrectionTest() {
    new Thread(odometer).start();
    new Thread(lsPoller).start();
    START[0] = 1;
    START[1] = 1;

    lsPoller.initializeLightSensor(LEFT_LIGHT_SENSOR);
    lsPoller.initializeLightSensor(RIGHT_LIGHT_SENSOR);

    LightPoller.startPolling(RIGHT_LIGHT_SENSOR);
    LightPoller.startPolling(LEFT_LIGHT_SENSOR);

    Navigation.setupMotors();
    double[] entrance = {2.5, 3.5};
    odometer.setXYT(TILE_SIZE, TILE_SIZE, 90);
    Navigation.navigateToTunnelEntrance(false, START, entrance);
  }

  /**
   * Method for testing localization after exiting a tunnel
   */
  private static void afterTunnelLocalizationTest() {
    new Thread(odometer).start();
    new Thread(lsPoller).start();

    lsPoller.initializeLightSensor(LEFT_LIGHT_SENSOR);
    lsPoller.initializeLightSensor(RIGHT_LIGHT_SENSOR);

    LightPoller.startLSPolling();

    Navigation.setupMotors();
    // Set starting position
    odometer.setXYT(2 * TILE_SIZE + 10, 2 * TILE_SIZE + 10, 280);
    LightLocalization.localizeAfterTunnel();
    System.out.print("\nx: " + odometer.getX() + "\ny: " + odometer.getY() + "\nt: " + odometer.getT());
  }

  /**
   * Method for testing localization before entering a tunnel
   */
  private static void beforeTunnelLocalizationTest() {
    new Thread(odometer).start();
    new Thread(lsPoller).start();


    lsPoller.initializeLightSensor(LEFT_LIGHT_SENSOR);
    lsPoller.initializeLightSensor(RIGHT_LIGHT_SENSOR);
    LightPoller.startLSPolling();

    Navigation.setupMotors();
    // Set starting position
    odometer.setXYT(2 * TILE_SIZE + 10, 2 * TILE_SIZE + 10, 280);
    LightLocalization.localizeBeforeTunnel();
    System.out.print("\nx: " + odometer.getX() + "\ny: " + odometer.getY() + "\nt: " + odometer.getT());
  }
}
