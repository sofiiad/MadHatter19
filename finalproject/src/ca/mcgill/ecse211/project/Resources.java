package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.wificlient.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.math.BigDecimal;
import java.util.Map;

/**
 * Resources class that offers all static final variables for ease of access. Defines all sensors and motors and their
 * ports. Integrates the WIFI class and its functionalities.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 * 
 * Wifi parameters reception.
 * @author Younes Boubekeur
 */
public class Resources {

  // Set these as appropriate for your team and current situation
  /**
   * The default server IP used by the profs and TA's.
   */
  public static final String DEFAULT_SERVER_IP = "192.168.2.53";

  /**
   * The IP address of the server that transmits data to the robot. Set this to the default for the beta demo and
   * competition.
   */
  public static final String SERVER_IP = DEFAULT_SERVER_IP;

  /**
   * Your team number.
   */
  public static final int TEAM_NUMBER = 19;

  /**
   * Enables printing of debug info from the WiFi class.
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /**
   * Enable this to attempt to receive Wi-Fi parameters at the start of the program.
   */
  public static final boolean RECEIVE_WIFI_PARAMS = true;

  // DECLARE YOUR CURRENT RESOURCES HERE

  //// Playing field variables//////
  /**
   * Start point for testing purposes
   */
  public static int[] START = {0, 0};

  /**
   * The playing field width
   */
  public static final int WIDTH = 15;

  /**
   * The playing field height
   */
  public static final int HEIGHT = 9;

  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;

  /**
   * Zone lower left
   */
  public static double[] ZONE_LL = new double[2];// = {0, 5}; // {10,0};//{0, 5};

  /**
   * Zone upper right
   */
  public static double[] ZONE_UR = new double[2];// = {4, 9}; // {15,4};//{4, 9};

  /**
   * Tunnel lower left
   */
  public static int[] TUNNEL_LL = new int[2];// = {4, 7}; // {10, 3};//{4, 7};

  /**
   * Tunnel upper right
   */
  public static int[] TUNNEL_UR = new int[2];// = {6, 8};// {11, 5};//{6, 8};

  /**
   * Island lower left
   */
  public static int[] ISLAND_LL = new int[2];// = {6, 5};

  /**
   * Island upper right
   */
  public static int[] ISLAND_UR = new int[2]; // = {15, 9};

  /**
   * The target for launching the ball
   */
  public static int[] TARGET = new int[2];// = {6, -3}; // {10, 13}; //CHECK FOR DOUBLE

  /**
   * The launching location
   */
  public static double[] launchPoint = new double[2];

  ///////// ROBOT HARDWARE VARIABLES ///////
  /**
   * Distance from axis of rotation to Light Sensor
   */
  public static final double AXIS_TO_LIGHT = 11.0; // Initially 16.4, 15.2 after tests

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.06; // Battery at 7.1, 2.13

  /**
   * The robot width in centimeters.
   */
  public static final double TRACK = 15.96; // 16.52; // Battery at 7.1, 16.8

  /**
   * The forward light sensor offset.
   */
  public static final int OFFSET = 4;

  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 150;


  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 170;

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 800;

  /**
   * The maximum launch distance in number of tiles
   */
  public static final int LAUNCH_THRESHOLD = 10;

  /**
   * Number of launches
   */
  public static final int NUMBER_OF_LAUNCHES = 10;

  /**
   * Offset of the launch arm in relation to the robot's turning base in cm
   */
  public static final int LAUNCH_OFFSET = 5;

  /**
   * Minimum launch distance per project constraints in cm
   */
  public static final double MINIMUM_LAUNCH_DISTANCE = 4 * TILE_SIZE;

  /**
   * Maximum precise launch distance per hardware constraints in cm
   */
  public static final double MAXIMUM_LAUNCH_DISTANCE = 12 * TILE_SIZE;

  ////////// THREAD VARIABLES///////////

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftLaunchMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightLaunchMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

  /**
   * The front ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor US_SENSOR_FRONT = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));

  /**
   * The right light sensor.
   */
  public static final EV3ColorSensor LEFT_LIGHT_SENSOR = new EV3ColorSensor(LocalEV3.get().getPort("S3"));

  /**
   * The left light sensor.
   */
  public static final EV3ColorSensor RIGHT_LIGHT_SENSOR = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();

  /**
   * The Ultrasonic poller.
   */
  public static UltrasonicPoller usPoller = new UltrasonicPoller();

  /**
   * The Light poller.
   */
  public static LightPoller lsPoller = new LightPoller();

  /**
   * The odometer correction.
   */
  public static OdometryCorrection odoCorrection = new OdometryCorrection();

  /**
   * Sampling rate of odometry correction
   */
  public static final long CORRECTION_PERIOD = 50;

  /**
   * Polling sample of light sensor
   */
  public static final long LS_WAIT = 10;

  /**
   * Polling sample of ultrasonic sensor
   */
  public static final long US_WAIT = 25;

  /**
   * Median filter for Ultrasonic polling, filters out noise
   */
  public static final int US_MEDIAN_FILTER = 5;

  /**
   * Maximum distance used to represent when there's nothing detected by the US sensor
   */
  public static final int MAXIMUM_DISTANCE = 300;

  /**
   * Maximum US sensor poller distance value for perceiving a wall.
   */
  public static final double INTERCEPT_DIST = 35;

  /**
   * Minimum US sensor poller distance value for re-detecting a wall.
   */
  public static final double WALL_DIST = 50;

  /**
   * Noise zone distance margin for perceiving a wall.
   */
  public static final double MARGIN_DIST = 5;



  //////////////////////////////////////

  /**
   * Container for the Wi-Fi parameters.
   */
  public static Map<String, Object> wifiParameters;

  // This static initializer MUST be declared before any Wi-Fi parameters.
  static {
    receiveWifiParameters();
  }

  /**
   * Red team number.
   */
  public static int redTeam = get("RedTeam");

  /**
   * Red team's starting corner.
   */
  public static int redCorner = get("RedCorner");

  /**
   * Green team number.
   */
  public static int greenTeam = get("GreenTeam");

  /**
   * Green team's starting corner.
   */
  public static int greenCorner = get("GreenCorner");

  /**
   * The Red Zone.
   */
  public static Region red = new Region("Red_LL_x", "Red_LL_y", "Red_UR_x", "Red_UR_y");

  /**
   * The Green Zone.
   */
  public static Region green = new Region("Green_LL_x", "Green_LL_y", "Green_UR_x", "Green_UR_y");

  /**
   * The Island.
   */
  public static Region island = new Region("Island_LL_x", "Island_LL_y", "Island_UR_x", "Island_UR_y");

  /**
   * The red tunnel footprint.
   */
  public static Region tnr = new Region("TNR_LL_x", "TNR_LL_y", "TNR_UR_x", "TNR_UR_y");

  // public static double targetTheta = Math.max(get("TNR_LL_x"), get("TNR_UR_x")); // only for beta

  /**
   * The green tunnel footprint.
   */
  public static Region tng = new Region("TNG_LL_x", "TNG_LL_y", "TNG_UR_x", "TNG_UR_y");

  /**
   * The location of the red target bin.
   */
  public static Point redBin = new Point(get("Red_BIN_x"), get("Red_BIN_y"));

  /**
   * The location of the green target bin.
   */
  public static Point greenBin = new Point(get("Green_BIN_x"), get("Green_BIN_y"));

  /**
   * Receives Wi-Fi parameters from the server program.
   */
  public static void receiveWifiParameters() {
    // Only initialize the parameters if needed
    if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
      return;
    }
    System.out.println("Waiting to receive Wi-Fi parameters.");

    // Connect to server and get the data, catching any errors that might occur
    try (WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button in the GUI on their
       * laptop with the data filled in. Once it's waiting, you can kill it by pressing the upper left hand corner
       * button (back/escape) on the EV3. getData() will throw exceptions if it can't connect to the server (e.g. wrong
       * IP address, server not running on laptop, not connected to WiFi router, etc.). It will also throw an exception
       * if it connects but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot will receive a
       * message saying an invalid team number was specified and getData() will throw an exception letting you know.
       */
      wifiParameters = conn.getData();
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }

  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   * 
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int get(String key) {
    if (wifiParameters != null) {
      return ((BigDecimal) wifiParameters.get(key)).intValue();
    } else {
      return 0;
    }
  }

  /**
   * Represents a region on the competition map grid, delimited by its lower-left and upper-right corners (inclusive).
   * 
   * @author Younes Boubekeur
   */
  public static class Region {
    /** The lower left corner of the region. */
    public Point ll;

    /** The upper right corner of the region. */
    public Point ur;

    /**
     * Constructs a Region.
     * 
     * @param lowerLeft the lower left corner of the region
     * @param upperRight the upper right corner of the region
     */
    public Region(Point lowerLeft, Point upperRight) {
      validateCoordinates(lowerLeft, upperRight);
      ll = lowerLeft;
      ur = upperRight;
    }

    /**
     * Helper constructor to make a Region directly from parameter names.
     * 
     * @param llX the Wi-Fi parameter key representing the lower left corner of the region x coordinate
     * @param llY the Wi-Fi parameter key representing the lower left corner of the region y coordinate
     * @param urX the Wi-Fi parameter key representing the upper right corner of the region x coordinate
     * @param urY the Wi-Fi parameter key representing the upper right corner of the region y coordinate
     */
    public Region(String llX, String llY, String urX, String urY) {
      this(new Point(get(llX), get(llY)), new Point(get(urX), get(urY)));
    }

    /**
     * Validates coordinates.
     * 
     * @param lowerLeft the lower left corner of the region
     * @param upperRight the upper right corner of the region
     */
    private void validateCoordinates(Point lowerLeft, Point upperRight) {
      if (lowerLeft.x > upperRight.x || lowerLeft.y > upperRight.y) {
        throw new IllegalArgumentException("Upper right cannot be below or to the left of lower left!");
      }
    }

    public String toString() {
      return "[" + ll + ", " + ur + "]";
    }
  }

  /**
   * Represents a coordinate point on the competition map grid.
   * 
   * @author Younes Boubekeur
   */
  public static class Point {
    /** The x coordinate. */
    public double x;

    /** The y coordinate. */
    public double y;

    /**
     * Constructs a Point.
     * 
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public String toString() {
      return "(" + x + ", " + y + ")";
    }

  }

}
