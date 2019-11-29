package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.OdometryCorrection.*;
import static ca.mcgill.ecse211.project.LightLocalization.localizeForward;
import static ca.mcgill.ecse211.project.LightLocalization.localizeBackward;
import static ca.mcgill.ecse211.project.Launcher.releaseLauncher;

/**
 * Class that offers static methods used for the robot's navigation. Most methods can be interrupted using the interrupt
 * flag.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 *
 */
public class Navigation {

  /**
   * Enumerator for tunnel orientation
   *
   */
  enum Orientation {
    NORTH, SOUTH, EAST, WEST
  }

  /**
   * Tunnel orientation
   */
  public static Orientation tunnelOri = Orientation.NORTH;

  /**
   * Flag to indicate that the robot is navigation
   */
  private static Navigation nav;

  /**
   * Boolean variable that will be used to indicate if the EV3 is turning or not.
   */
  public static boolean turning;

  /**
   * Boolean variable that will be used to indicate if EV3 is still navigating through the course.
   */
  private static boolean navigating;

  /**
   * Instantiating interrupt variable to false. This indicates that there is no initial obstacle in front of the EV3.
   */
  private static boolean interrupt = false;

  /**
   * Last x position the robot was at
   */
  private static double last_X;

  /**
   * Last y position the robot was at
   */
  private static double last_Y;

  /**
   * Entrance of the tunnel as the middle coordinates in front of the tunnel's entrance.
   */
  public static double[] tileEntrance = new double[2];

  /**
   * Exit of the tunnel as the middle coordinates in front of the tunnel's exit.
   */
  public static double[] tileExit = new double[2];

  /**
   * Boolean relation between the higher and lower coordinates of the tunnel and island, false is lower
   */
  public static boolean relationOffset = true;

  /**
   * Boolean for whether the tunnel is horizontal(enter and exit on X) or vertical(enter and exit on Y)
   */
  public static boolean isHorizontalTunnel = true;

  /**
   * Entrance of the tunnel as a tile intersection point
   */
  public static int[] tunnelEntrance = new int[2];

  /**
   * Exit of the tunnel as a tile intersection point
   */
  public static int[] tunnelExit = new int[2];

  /**
   * Stores int for island lower bound in X
   */
  private static int xLow = 0;

  /**
   * Stores int for island upper bound in X
   */
  private static int xHigh = 0;

  /**
   * Stores int for island lower bound in Y
   */
  private static int yLow = 0;

  /**
   * Store int for island upper bound in Y
   */
  private static int yHigh = 0;

  /**
   * Stores int for iterating through constructed tiles within playing field
   */
  private static int tileCount = 1;

  /**
   * Stores boolean marking the identification of a suitable launch point
   */
  private static boolean tileFound = false;

  /**
   * Tile height of the island
   */
  public static int height = ISLAND_UR[1] - ISLAND_LL[1];

  /**
   * Tile width of the island
   */
  public static int width = ISLAND_UR[0] - ISLAND_LL[0];

  /**
   * Number of tiles within the island
   */
  public static int tiles = height * width;

  /**
   * Iterable 2D array for constructing the playing field and traversing its data
   */
  public static double areaTiles[][] = new double[tiles][2];


  /**
   * Set up motors for initial start
   */
  public synchronized static void setupMotors() {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.resetTachoCount();
        rightMotor.resetTachoCount();
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
      }
    }
  }

  /**
   * Sets the motor speeds jointly.
   */
  public synchronized static void setSpeeds(float leftSpeed, float rightSpeed) {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);
      }
    }
    if (leftSpeed < 0) {
      leftMotor.backward();
    } else {
      leftMotor.forward();
    }
    if (rightSpeed < 0) {
      rightMotor.backward();
    } else {
      rightMotor.forward();
    }
  }

  /**
   * Travels to designated position.
   * 
   * @param x
   * @param y
   */
  static synchronized void travelTo(double x, double y) {

    navigating = true;
    last_X = odometer.getX();
    last_Y = odometer.getY();
    double dX = x - last_X;
    double dY = y - last_Y;
    Main.sleepFor(30);
    turnTo(Math.toDegrees(Math.atan2(dX, dY))); // turn to right direction
    Main.sleepFor(30);
    travel(dX, dY);
  }

  /**
   * Turns to desired position
   * 
   * @param x
   * @param y
   */
  static synchronized void turnTo(double x, double y) {
    last_X = odometer.getX();
    last_Y = odometer.getY();
    double dX = x - last_X;
    double dY = y - last_Y;

    turnTo(Math.toDegrees(Math.atan2(dX, dY))); // turn to right direction

  }

  /**
   * The robot travels a certain amount of cm given coordinates
   * 
   * @param dX absolute displacement in cm
   * @param dY absolute displacement in cm
   */
  static synchronized void travel(double dX, double dY) {
    travel(Math.sqrt(dX * dX + dY * dY));

  }

  /**
   * Travels a certain amount of cm given distance
   * 
   * @param distance in cm
   */
  static synchronized void travel(double distance) {
    double dX, dY;
    last_X = odometer.getX();
    last_Y = odometer.getY();
    travelForward();

    while (!isInterrupt()) {// While no obstacle detected
      dX = Math.abs(odometer.getX() - last_X);
      dY = Math.abs(odometer.getY() - last_Y);
      if (dX * dX + dY * dY >= distance * distance) {// reached destination
        break;
      }
    }

    stopMotors();
    navigating = false;
  }

  /**
   * Travels forward without stopping
   */
  static synchronized void travelForward() {
    navigating = true;
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        rightMotor.forward();
        leftMotor.forward();
      }
    }
    Main.sleepFor(50);
    navigating = false;
  }

  /**
   * Turns to specified angle (in degrees) by respecting the minimum turning angle
   * 
   * @param theta angles in degrees
   */
  public synchronized static void turnTo(double theta) {

    double odoTheta = odometer.getT();

    double rotation = (theta - odoTheta);

    if (rotation > 180) {
      rotation = rotation - 360;

    } else if (rotation < -180) {
      rotation = rotation + 360;

    } else if (Math.abs(rotation) == 180) {
      rotation = Math.abs(rotation);
    }

    turn(rotation);
  }

  /**
   * Turn clockwise by a certain amount of degrees
   * 
   * @param rotation clockwise in degrees
   * 
   */
  public synchronized static void turn(double rotation) {

    turning = true;
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        leftMotor.rotate(convertAngle(rotation), true);
        rightMotor.rotate(-convertAngle(rotation), false);
      }
    }
    turning = false;
  }

  /**
   * Immediate return turn to specified angle (in degrees) by respecting the minimum turning angle
   * 
   * @param theta angles in degrees
   */
  public synchronized static void turnTo(double theta, boolean immediate) {

    double odoTheta = odometer.getT();

    double rotation = (theta - odoTheta);

    if (rotation > 180) {
      rotation = rotation - 360;

    } else if (rotation < -180) {
      rotation = rotation + 360;

    } else if (Math.abs(rotation) == 180) {
      rotation = Math.abs(rotation);
    }

    turn(rotation, immediate);
  }

  /**
   * Immediate return turn clockwise by a certain amount of degrees
   * 
   * @param rotation clockwise in degrees
   * 
   */
  public synchronized static void turn(double rotation, boolean immediate) {

    turning = true;
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        leftMotor.rotate(convertAngle(rotation), true);
        rightMotor.rotate(-convertAngle(rotation), immediate);
      }
    }
  }

  /**
   * Stops the robot.
   */
  public synchronized static void stopMotors() {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        rightMotor.setSpeed(0);
        leftMotor.setSpeed(0);
        rightMotor.stop();
        leftMotor.stop();
      }
    }
    navigating = false;
  }

  /**
   * Moves robot forward a set distance in cm. Used for navigation without obstacle avoidance setting.
   * 
   * @param distance in cm
   * 
   */
  public synchronized static void moveForward(double distance) {

    navigating = true;
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.forward();
        rightMotor.forward();
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        leftMotor.rotate(convertDistance(Math.abs(distance)), true);
        rightMotor.rotate(convertDistance(Math.abs(distance)), false);
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
      }
    }
    navigating = false;
  }

  /**
   * Moves robot forward a set distance in cm. Used for navigation with obstacle avoidance setting.
   * 
   * @param distance in cm
   * 
   */
  public synchronized static void moveForward(double distance, boolean immediateReturn) {

    navigating = true;

    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate(convertDistance(distance), true);
        rightMotor.rotate(convertDistance(distance), immediateReturn);
      }
    }
  }

  /**
   * Set motors and move robot forward at low speed for light sensor detection. Used in LightLocalization
   */
  public static void moveForwardDetection() {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(90);
        rightMotor.setSpeed(90);
        leftMotor.forward();
        rightMotor.forward();
      }
    }
  }

  /**
   * Set motors and move robot backward at low speed for light sensor detection. Used in LightLocalization
   */
  public static void moveBackwardDetection() {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(90);
        rightMotor.setSpeed(90);
        leftMotor.backward();
        rightMotor.backward();
      }
    }
  }

  /**
   * Turn robot 90 degrees
   */
  public static void turnRight() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    Main.sleepFor(30);
    turn(90);
    Main.sleepFor(30);
    stopMotors();
    Main.sleepFor(30);
  }

  /**
   * Travels a certain amount of cm given distance in the reverse direction (backwards)
   * 
   * @param distance
   */
  public synchronized static void travelBack(double distance) {
    double dX, dY;
    last_X = odometer.getX();
    last_Y = odometer.getY();
    travelBackward();

    while (!isInterrupt()) {// While no obstacle detected
      dX = Math.abs(odometer.getX() - last_X);
      dY = Math.abs(odometer.getY() - last_Y);
      if (dX * dX + dY * dY >= distance * distance) {// reached destination
        break;
      }
    }

    stopMotors();
    navigating = false;
  }

  /**
   * Moves robot backward until a stop is called
   */
  public synchronized static void travelBackward() {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        leftMotor.backward();
        rightMotor.backward();
      }
    }
  }

  /**
   * Returns navigation status.
   * 
   * @return navigating True during navigation
   */
  public static boolean isNavigating() {
    return navigating;
  }

  /**
   * Sets navigation status
   * 
   * @param navigating
   */
  public static void setNavigating(boolean navigating) {
    Navigation.navigating = navigating;
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance in cm
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * Returns the destination angle.
   * 
   * @param x
   * @param y
   * @return minAngle The destination angle.
   */
  public static double getDestAngle(double x, double y) {

    double minAngle;

    last_X = odometer.getX();
    last_Y = odometer.getY();
    double dX = x - last_X;
    double dY = y - last_Y;

    minAngle = Math.toDegrees(Math.atan2(dX, dY)); // turn to right direction

    return minAngle;
  }


  /**
   * Returns the distance in cm between the robot and its next waypoint.
   * 
   * @return distance between robot and next waypoint in cm.
   */
  public static double distanceToNextWayPoint(double dest_X, double dest_Y) {

    double distance;

    double current_X = odometer.getX();
    double current_Y = odometer.getY();

    double dX = current_X - dest_X;
    double dY = current_Y - dest_Y;

    distance = Math.sqrt(Math.pow(dX, 2) - Math.pow(dY, 2));

    return distance;

  }

  /**
   * Rotate continuously in one direction
   * 
   * @param clockwise Direction which the robot needs to turn CLOCK_WISE is true, COUNTER_CLOCK_WISE is false
   * 
   */
  public synchronized static void rotate(boolean clockwise) {
    synchronized (leftMotor) {
      synchronized (rightMotor) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
      }
    }

    if (clockwise) { // turn clockwise
      synchronized (leftMotor) {
        synchronized (rightMotor) {
          rightMotor.backward();
          leftMotor.forward();
        }
      }
    } else { // turn counter clockwise
      synchronized (leftMotor) {
        synchronized (rightMotor) {
          rightMotor.forward();
          leftMotor.backward();
        }
      }
    }
  }

  /**
   * Returns the Navigation Object. Use this method to obtain an instance of Navigation.
   * 
   * @return the Navigation Object
   */
  public synchronized static Navigation getNavigation() {
    if (nav == null) {
      nav = new Navigation();
    }
    return nav;
  }

  /**
   * Returns interrupt status (has it detected an obstacle or not)
   * 
   * @return interrupt
   * 
   */
  public static boolean isInterrupt() {
    return interrupt;
  }

  /**
   * Sets interrupt state
   */
  public static void setInterrupt(boolean interrupt) {
    Navigation.interrupt = interrupt;
  }

  public synchronized static void navigateToPoint(double currentPoint[], double targetPoint[]) {

    double distXToPoint;
    double distYToPoint;

    distXToPoint = targetPoint[0] - currentPoint[0];
    distYToPoint = targetPoint[1] - currentPoint[1];

    if (Math.abs(distXToPoint) >= Math.abs(distYToPoint)) {
      travelTo(targetPoint[0] * TILE_SIZE, currentPoint[1] * TILE_SIZE);
    } else {
      travelTo(currentPoint[0] * TILE_SIZE, targetPoint[1] * TILE_SIZE);
    }
    travelTo(targetPoint[0] * TILE_SIZE, targetPoint[1] * TILE_SIZE);
    Main.sleepFor(30);
  }

  /**
   * Navigation to specified point split by axis for int[] destination
   * 
   * @param currentPoint Current point tile coordinates
   * @param targetPoint  Target location in tile coordinates
   */
  public synchronized static void navigateToPoint(double currentPoint[], int targetPoint[]) {

    double distXToPoint;
    double distYToPoint;

    distXToPoint = targetPoint[0] - currentPoint[0];
    distYToPoint = targetPoint[1] - currentPoint[1];

    if (Math.abs(distXToPoint) >= Math.abs(distYToPoint)) {
      travelTo(targetPoint[0] * TILE_SIZE, currentPoint[1] * TILE_SIZE);
    } else {
      travelTo(currentPoint[0] * TILE_SIZE, targetPoint[1] * TILE_SIZE);
    }
    travelTo(targetPoint[0] * TILE_SIZE, targetPoint[1] * TILE_SIZE);
  }

  /**
   * Navigation through a tunnel after lowering launch arm
   */
  public static void navigateThroughTunnel(double[] exit) {
    leftLaunchMotor.flt();
    rightLaunchMotor.flt();
    leftLaunchMotor.setSpeed(40);
    rightLaunchMotor.setSpeed(40);
    leftLaunchMotor.rotate(62, true);
    rightLaunchMotor.rotate(62, false);
    leftLaunchMotor.setSpeed(0);
    rightLaunchMotor.setSpeed(0);

    Main.sleepFor(50);
    travelTo(exit[0] * TILE_SIZE, exit[1] * TILE_SIZE);

    Main.sleepFor(50);
    moveForward(OFFSET * 2);
    Main.sleepFor(50);
    releaseLauncher();
    Main.sleepFor(50);
  }

  /**
   * Navigation through a tunnel after lowering launch arm
   */
  public static void navigateBackThroughTunnel(double[] exit) {
    turnTo(exit[0] * TILE_SIZE, exit[1] * TILE_SIZE);
    localizeBackward();

    leftLaunchMotor.flt();
    rightLaunchMotor.flt();
    leftLaunchMotor.setSpeed(40);
    rightLaunchMotor.setSpeed(40);
    leftLaunchMotor.rotate(62, true);
    rightLaunchMotor.rotate(62, false);
    leftLaunchMotor.setSpeed(0);
    rightLaunchMotor.setSpeed(0);

    travelTo(exit[0] * TILE_SIZE, exit[1] * TILE_SIZE);
    localizeForward();
    leftLaunchMotor.flt();
    rightLaunchMotor.flt();
  }

  /**
   * Navigate to tunnel entrance
   * 
   * @param startPoint Tile location
   * @param tunnelLL Lower left coordinate of tunnel 
   * @param tunnelUR Upper right coordinate of tunnel
   */
  public static void toTunnelEntrance(int startPoint[], int tunnelLL[], int tunnelUR[]) {

    setTunnelOrientation(tunnelLL, tunnelUR);

    setTunnelOpenings(startPoint, tunnelLL, tunnelUR);

    convertTunnelOpeningToTile(isHorizontalTunnel, startPoint, tunnelEntrance, tunnelExit);

    navigateToTunnelEntrance(isHorizontalTunnel, startPoint, tileEntrance);

  }

  /**
   * Navigate to optimal launch point from current tile
   * 
   * @param areaLower Lower playable area
   * @param areaUpper Upper playable area
   * @param currentTile Current tile location
   * @param bin Target bin location
   */
  public synchronized static void toLaunchPointFromCurrentTile(int areaLower[], int areaUpper[], double currentTile[],
      int bin[]) {

    setupAreaConstructor(relationOffset, isHorizontalTunnel, currentTile);

    constructTilesAroundRobotPosition(relationOffset, isHorizontalTunnel, areaLower, areaUpper, currentTile, TARGET);

    navigateToPoint(tileExit, launchPoint);

  }

  /**
   * Set isHorizontalTunnel: horizontal is true, vertical is false
   * 
   * @param tunnelLL LowerLeft tunnel parameter
   * @param tunnelUR UpperRight tunnel parameter
   */
  public static void setTunnelOrientation(int tunnelLL[], int tunnelUR[]) {
    int dTunnelX;
    int dTunnelY;

    dTunnelX = Math.abs(tunnelUR[0] - tunnelLL[0]);
    dTunnelY = Math.abs(tunnelUR[1] - tunnelLL[1]);

    if (dTunnelX > dTunnelY) {
      isHorizontalTunnel = true;
    } else if (dTunnelX < dTunnelY) {
      isHorizontalTunnel = false;

    } else {
      // Case where tunnel has length 1 tile
      // Top tile of tunnel within island
      if (tunnelLL[0] >= ISLAND_LL[0] && (tunnelLL[1] + 1) >= ISLAND_LL[1] && tunnelLL[0] < ISLAND_UR[0]
          && (tunnelLL[1] + 1) < ISLAND_UR[1]) {
        // Bottom tile of tunnel within zone
        if (tunnelLL[0] < ZONE_UR[0] && (tunnelLL[1] - 1) < ZONE_UR[1] && tunnelLL[0] >= ZONE_LL[0]
            && (tunnelLL[1] - 1) >= ZONE_LL[1]) {
          isHorizontalTunnel = false;
        }
      }

      // Top tile of tunnel within zone
      if (tunnelLL[0] >= ZONE_LL[0] && (tunnelLL[1] + 1) >= ZONE_LL[1] && tunnelLL[0] < ZONE_UR[0]
          && (tunnelLL[1] + 1) < ZONE_UR[1]) {
        // Bottom tile of tunnel within island
        if (tunnelLL[0] < ISLAND_UR[0] && (tunnelLL[1] - 1) < ISLAND_UR[1] && tunnelLL[0] >= ISLAND_LL[0]
            && (tunnelLL[1] - 1) >= ISLAND_LL[1]) {
          isHorizontalTunnel = false;
        }
      }

      // If vertical conditions not met, tunnel must be horizontal
      isHorizontalTunnel = true;
    }
  }

  /**
   * Set tunnelEntrance and tunnelExit
   * 
   * @param startPoint Initial robot position
   * @param tunnelLL LowerLeft tunnel parameter
   * @param tunnelUR UpperRight tunnel parameter
   */
  public static void setTunnelOpenings(int startPoint[], int tunnelLL[], int tunnelUR[]) {
    double distXToTunnel1;
    double distXToTunnel2;
    double distYToTunnel1;
    double distYToTunnel2;

    double distToTunnel1;
    double distToTunnel2;

    distXToTunnel1 = Math.abs(startPoint[0] - tunnelLL[0]);
    distXToTunnel2 = Math.abs(startPoint[0] - tunnelUR[0]);
    distYToTunnel1 = Math.abs(startPoint[1] - tunnelLL[1]);
    distYToTunnel2 = Math.abs(startPoint[1] - tunnelUR[1]);

    distToTunnel1 = Math.hypot(distXToTunnel1, distYToTunnel1);
    distToTunnel2 = Math.hypot(distXToTunnel2, distYToTunnel2);

    if (distToTunnel1 > distToTunnel2) {
      tunnelEntrance = tunnelUR;
      tunnelExit = tunnelLL;
    } else {
      tunnelEntrance = tunnelLL;
      tunnelExit = tunnelUR;
    }
  }

  /**
   * Convert tunnel opening coordinates to middle of tile references
   * 
   * @param isHorizontalTunnel True if tunnel is horizontal
   * @param startPoint Tile coordinate start point
   * @param tunnelEntrance Tile coordinate entrance 
   * @param tunnelExit Tile coordinate exit
   */
  public static void convertTunnelOpeningToTile(boolean isHorizontalTunnel, int[] startPoint, int[] tunnelEntrance,
      int[] tunnelExit) {

    if (!isHorizontalTunnel) {
      tileEntrance[0] = (double) (tunnelEntrance[0] + tunnelExit[0]) / 2;
      tileExit[0] = tileEntrance[0];

      setYRelationOffsetToTunnel(startPoint, tunnelEntrance);

      if (relationOffset) {
        tileEntrance[1] = tunnelEntrance[1] - 0.5;
        tileExit[1] = tunnelExit[1] + 0.5;
      } else if (!relationOffset) {
        tileEntrance[1] = tunnelEntrance[1] + 0.5;
        tileExit[1] = tunnelExit[1] - 0.5;
      }

    } else {
      tileEntrance[1] = (double) (tunnelEntrance[1] + tunnelExit[1]) / 2;
      tileExit[1] = tileEntrance[1];

      setXRelationOffsetToTunnel(startPoint, tunnelEntrance);

      if (relationOffset) {
        tileEntrance[0] = tunnelEntrance[0] - 0.5;
        tileExit[0] = tunnelExit[0] + 0.5;
      } else if (!relationOffset) {
        tileEntrance[0] = tunnelEntrance[0] + 0.5;
        tileExit[0] = tunnelExit[0] - 0.5;
      }
    }
  }

  /**
   * Directional X relation from startPoint to tunnelOpening Set relationOffset: closer entrance is true, closer exit is
   * false
   * 
   * @param startPoint Tile coordinate start point
   * @param tunnelOpening Tile coordinate tunnel opening
   */
  public static void setXRelationOffsetToTunnel(int[] startPoint, int[] tunnelOpening) {
    if (tunnelOpening[0] > startPoint[0]) {
      relationOffset = true; // Left half
    } else {
      relationOffset = false; // Right half
    }
  }


  /**
   * Directional Y relation from startPoint to tunnelOpening Set relationOffset: closer entrance is true, closer exit is
   * false
   * 
   * @param startPoint Tile coordinate start point
   * @param tunnelOpening Tile coordinate tunnel opening
   */
  public static void setYRelationOffsetToTunnel(int[] startPoint, int[] tunnelOpening) {
    if (tunnelOpening[1] > startPoint[1]) {
      relationOffset = true; // Lower half
    } else {
      relationOffset = false; // Upper half
    }
  }

  /**
   * Logic for navigating to the entrance of a tunnel with odometry correction. Covers the distance perpendicular to the
   * tunnel's opening first to limit zone edge cases.
   * 
   * @param horizontal Direction of tunnel
   * @param start Starting point
   * @param dest Destination point is the entrance of the tunnel
   */
  public static void navigateToTunnelEntrance(boolean horizontal, int[] start, double[] dest) {
    boolean isHigherX = false;
    boolean isHigherY = false;
    double offLineX = 0.5;
    double offLineY = 0.5;
    double oneOffX = 1;
    double oneOffY = 1;

    if (dest[0] > start[0]) {
      oneOffX = -1;
      offLineX = -0.5;
    } else if (dest[0] < start[0]) {
      isHigherX = true;
    } else {

    }

    if (dest[1] > start[1]) {
      oneOffY = -1;
      offLineY = -0.5;
    } else if (dest[1] < start[1]) {
      isHigherY = true;
    } else {

    }

    int xPos = start[0], yPos = start[1];
    if (!horizontal) {
      if (!isHigherX) {
        // Move from lower X to higher X with correction
        for (xPos = start[0]; xPos + 3 < dest[0];) {
          xPos += 2;
          Main.sleepFor(50);
          travelTo((xPos - offLineX) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
          xPos += 1;
          correct(xPos * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
        }
        if (dest[0] > xPos - oneOffX) {
          Main.sleepFor(50);
          travelTo((dest[0] + oneOffX) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
          correct(Math.floor(dest[0]) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
        }
      } else {

        // Move from higher X to lower X with correction
        for (xPos = start[0]; xPos - 3 > Math.floor(dest[0]);) {
          xPos -= 2;
          System.out.print("\n xPos: " + xPos);
          Main.sleepFor(50);
          travelTo((xPos - offLineX) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
          xPos -= 1;
          correct(xPos * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
        }
        if (dest[0] < xPos - oneOffX) {
          Main.sleepFor(50);
          travelTo((dest[0] + oneOffX) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
          correct(Math.ceil(dest[0]) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);

        }
      }
      // Reach X coordinate of entrance
      Main.sleepFor(100);
      if (dest[0] != (int) dest[0] || dest[0] != xPos) {
        travelTo(dest[0] * TILE_SIZE, odometer.getY());
        Main.sleepFor(50);
      }

      if (!isHigherY) {
        // Move from lower Y to higher Y with correction at every tile
        tunnelOri = Orientation.NORTH;
        for (yPos = start[1]; yPos < Math.floor(dest[1]);) {
          yPos += 1;
          Main.sleepFor(50);
          travelTo(odometer.getX(), (yPos + offLineY) * TILE_SIZE);
          Main.sleepFor(50);
          correct(odometer.getX(), yPos * TILE_SIZE);
          Main.sleepFor(50);
          System.out.print("\n X: " + odometer.getX() + " Y: " + odometer.getY());
        }
      } else {
        // Move from higher Y to lower Y with correction at every tile
        tunnelOri = Orientation.SOUTH;
        for (yPos = start[1]; yPos > Math.ceil(dest[1]);) {
          yPos -= 1;
          Main.sleepFor(50);
          travelTo(odometer.getX(), (yPos + offLineY) * TILE_SIZE);
          Main.sleepFor(50);
          correct(odometer.getX(), yPos * TILE_SIZE);
          Main.sleepFor(50);
        }
      }
      // Reach Y coordinate of entrance
      Main.sleepFor(100);
      if (dest[1] != (int) dest[1] || dest[1] != yPos) {
        travelTo(odometer.getX(), dest[1] * TILE_SIZE);
        Main.sleepFor(50);
      }
    } else {
      if (!isHigherY) {
        // Move from lower Y to higher Y with correction
        for (yPos = start[1]; yPos + 3 < dest[1];) {
          yPos += 2;
          Main.sleepFor(50);
          travelTo(odometer.getX(), (yPos - offLineY) * TILE_SIZE);
          Main.sleepFor(50);
          yPos += 1;
          correct(odometer.getX(), yPos * TILE_SIZE);
          Main.sleepFor(50);
        }
        if (dest[1] > yPos - oneOffY) {
          Main.sleepFor(50);
          travelTo(odometer.getX(), (dest[1] + oneOffY) * TILE_SIZE);
          Main.sleepFor(50);
          correct(odometer.getX(), Math.floor(dest[1]) * TILE_SIZE);
          Main.sleepFor(100);
        }
      } else {
        // Move from higher Y to lower Y with correction
        for (yPos = start[1]; yPos - 3 > Math.floor(dest[1]);) {
          yPos -= 2;
          Main.sleepFor(50);
          travelTo(odometer.getX(), (yPos - offLineY) * TILE_SIZE);
          Main.sleepFor(50);
          yPos -= 1;
          correct(odometer.getX(), yPos * TILE_SIZE);
          Main.sleepFor(50);
        }
        if (dest[1] < yPos - oneOffY) {
          Main.sleepFor(50);
          travelTo(odometer.getX(), (dest[1] + oneOffY) * TILE_SIZE);
          Main.sleepFor(50);
          correct(odometer.getX(), Math.ceil(dest[1]) * TILE_SIZE);
          Main.sleepFor(100);
        }
      }
      // Reach Y coordinate of entrance
      Main.sleepFor(100);
      if (dest[1] != (int) dest[1] || dest[1] != yPos) {
        travelTo(odometer.getX(), dest[1] * TILE_SIZE);
        Main.sleepFor(50);
      }

      if (!isHigherX) {
        // Move from lower X to higher X with correction at every tile
        tunnelOri = Orientation.EAST;
        for (xPos = start[0]; xPos < Math.floor(dest[0]);) {
          xPos += 1;
          Main.sleepFor(50);
          travelTo((xPos + offLineX) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
          correct(xPos * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
        }
      } else {
        // Move from higher X to lower X with correction at every tile
        tunnelOri = Orientation.WEST;
        for (xPos = start[0]; xPos > Math.ceil(dest[0]);) {
          xPos -= 1;
          Main.sleepFor(50);
          travelTo((xPos + offLineX) * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
          correct(xPos * TILE_SIZE, odometer.getY());
          Main.sleepFor(50);
        }
      }
      // Reach X coordinate of entrance
      Main.sleepFor(100);
      if (dest[0] != (int) dest[0] || dest[0] != xPos) {
        travelTo(dest[0] * TILE_SIZE, odometer.getY());
        Main.sleepFor(50);
      }
    }
  }

  /**
   * Set iterators for constructing tiles within playing area
   * 
   * @param relationOffset True if lower on any axis
   * @param isHorizontalTunnel True if tunnel is horizontal
   * @param currentTile Tile point of current position
   */
  public static void setupAreaConstructor(boolean relationOffset, boolean isHorizontalTunnel, double[] currentTile) {
    if (relationOffset && isHorizontalTunnel) {
      yHigh = (int) Math.floor(currentTile[1]);
      yLow = yHigh;

      xLow = (int) Math.floor(currentTile[0]);
      xHigh = xLow + 1;

    } else if (!relationOffset && isHorizontalTunnel) {
      yHigh = (int) Math.floor(currentTile[1]);
      yLow = yHigh;

      xHigh = (int) Math.floor(currentTile[0]);;
      xLow = xHigh - 1;

    } else if (relationOffset && !isHorizontalTunnel) {
      yLow = (int) Math.floor(currentTile[1]);
      yHigh = yLow + 1;

      xLow = (int) Math.floor(currentTile[0]);
      xHigh = xLow;

    } else if (!relationOffset && !isHorizontalTunnel) {
      yHigh = (int) Math.floor(currentTile[1]);
      yLow = yHigh - 1;

      xHigh = (int) Math.floor(currentTile[0]);;
      xLow = xHigh;
    }
  }

  /**
   * Handles cases for setting optimal launch tile from the construction of tiles near robot position
   * 
   * @param relationOffset True if lower on any axis
   * @param isHorizontalTunnel True if tunnel is horizontal
   * @param areaLower Array area
   * @param areaUpper Array area
   * @param currentTile Tile point of current position
   * @param bin Target bin
   */
  public static void constructTilesAroundRobotPosition(boolean relationOffset, boolean isHorizontalTunnel,
      int[] areaLower, int[] areaUpper, double[] currentTile, int[] bin) {

    areaTiles[0][0] = currentTile[0];
    areaTiles[0][1] = currentTile[1];
    int xInnerHigh = xHigh;
    int xInnerLow = xLow;
    int yInnerHigh = yHigh;
    int yInnerLow = yLow;

    while (!tileFound) {
      if (relationOffset && isHorizontalTunnel) {
        // Case where more to fill up
        if (yHigh < (areaUpper[1] - 2)) {
          yHigh++; // Will reach max areaUpper[1] - 2

          // Case for fix y-axis tiles above tunnel start
          for (int i = xInnerLow; i <= xInnerHigh && !tileFound; i++) {
            areaTiles[tileCount][0] = i + 0.5;
            areaTiles[tileCount][1] = yHigh + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill down
        if (yLow > (areaLower[1] + 1) && !tileFound) {
          yLow--; // Will reach min areaLower[1] + 1

          // Case for fix y-axis tiles below tunnel start
          for (int i = xInnerLow; i <= xInnerHigh && !tileFound; i++) {
            areaTiles[tileCount][0] = i + 0.5;
            areaTiles[tileCount][1] = yLow + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill on X
        if (xHigh < (areaUpper[0] - 2) && !tileFound) {
          xHigh++; // Will reach max areaUpper[0] - 2

          // Case for fix x-axis tiles in front of tunnel start
          for (int i = yInnerHigh; i >= yInnerLow && !tileFound; i--) {
            areaTiles[tileCount][0] = xHigh + 0.5;
            areaTiles[tileCount][1] = i + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }
      } else if (!relationOffset && isHorizontalTunnel) {
        if (yHigh < (areaUpper[1] - 2)) {
          yHigh++; // Will reach max areaUpper[1] - 2

          // Case for fix y-axis tiles above tunnel start
          for (int i = xInnerHigh; i >= xInnerLow && !tileFound; i--) {
            areaTiles[tileCount][0] = i + 0.5;
            areaTiles[tileCount][1] = yHigh + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill down
        if (yLow > (areaLower[1] + 1) && !tileFound) {
          yLow--; // Will reach min areaLower[1] + 1

          // Case for fix y-axis tiles below tunnel start
          for (int i = xInnerHigh; i >= xInnerLow && !tileFound; i--) {
            areaTiles[tileCount][0] = i + 0.5;
            areaTiles[tileCount][1] = yLow + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill on X
        if (xHigh > (areaLower[0] + 1) && !tileFound) {
          xHigh--; // Will reach min areaLower[0] + 1

          // Case for fix x-axis tiles in front of tunnel start
          for (int i = yInnerHigh; i >= yInnerLow && !tileFound; i--) {
            areaTiles[tileCount][0] = xHigh + 0.5;
            areaTiles[tileCount][1] = i + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }
      } else if (relationOffset && !isHorizontalTunnel) {
        // Case where more to fill left
        if (xLow > (areaLower[0] + 1)) {
          xLow--; // Will reach min areaLower[0] + 1

          // Case for fix x-axis tiles left parallel to tunnel start
          for (int i = yInnerLow; i <= yInnerHigh && !tileFound; i++) {
            areaTiles[tileCount][0] = xLow + 0.5;
            areaTiles[tileCount][1] = i + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill right
        if (xHigh < (areaUpper[0] - 2) && !tileFound) {
          xHigh++; // Will reach max areaUpper[0] - 2

          // Case for fix x-axis tiles right parallel to tunnel start
          for (int i = yInnerLow; i <= yInnerHigh && !tileFound; i++) {
            areaTiles[tileCount][0] = xHigh + 0.5;
            areaTiles[tileCount][1] = i + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill on y
        if (yHigh < (areaUpper[1] - 2) && !tileFound) {
          yHigh++; // Will reach max areaUpper[1] - 2

          // Case for fix y-axis tiles in front of tunnel start
          for (int i = xInnerLow; i <= xInnerHigh && !tileFound; i++) {
            areaTiles[tileCount][0] = i + 0.5;
            areaTiles[tileCount][1] = yHigh + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }
      } else if (!relationOffset && !isHorizontalTunnel) {
        // Case where more to fill left
        if (xLow > (areaLower[0] + 1)) {
          xLow--; // Will reach min areaLower[0] + 1

          // Case for fix x-axis tiles left parallel to tunnel start
          for (int i = yInnerHigh; i >= yInnerLow && !tileFound; i--) {
            areaTiles[tileCount][0] = xLow + 0.5;
            areaTiles[tileCount][1] = i + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill right
        if (xHigh < (areaUpper[0] - 2) && !tileFound) {
          xHigh++; // Will reach max areaUpper[0] - 2

          // Case for fix x-axis tiles right parallel to tunnel start
          for (int i = yInnerHigh; i >= yInnerLow && !tileFound; i--) {
            areaTiles[tileCount][0] = xHigh + 0.5;
            areaTiles[tileCount][1] = i + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }

        // Case where more to fill on y
        if (yHigh > (areaLower[1] + 1) && !tileFound) {
          yHigh--; // Will reach min areaLower[1] + 1

          // Case for fix y-axis tiles in front of tunnel start
          for (int i = xInnerLow; i <= xInnerHigh && !tileFound; i++) {
            areaTiles[tileCount][0] = i + 0.5;
            areaTiles[tileCount][1] = yHigh + 0.5;

            tileFound = setLaunchTile(bin, areaTiles, tileCount);
            tileCount++;
          }
        }
      }
    }
  }

  /**
   * Set launch tile and return true if found
   * 
   * @param bin Target bin
   * @param areaTiles 2D array construction of area
   * @param tileCount Iterator
   * @return localTileFound True if suitable launch tile position is found
   */
  public static boolean setLaunchTile(int bin[], double areaTiles[][], int tileCount) {

    boolean localTileFound = false;
    double distTileToLaunch =
        Math.sqrt(Math.pow(bin[0] - areaTiles[tileCount][0], 2) + Math.pow(bin[1] - areaTiles[tileCount][1], 2));


    if (distTileToLaunch < LAUNCH_THRESHOLD) {
      launchPoint[0] = areaTiles[tileCount][0];
      launchPoint[1] = areaTiles[tileCount][1];
      localTileFound = true;
    }

    return localTileFound;
  }
}
