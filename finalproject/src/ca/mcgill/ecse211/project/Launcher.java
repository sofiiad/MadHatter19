package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.travelBack;
import static ca.mcgill.ecse211.project.Navigation.turnTo;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.OFFSET;
import static ca.mcgill.ecse211.project.Resources.NUMBER_OF_LAUNCHES;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.LAUNCH_OFFSET;
import static ca.mcgill.ecse211.project.Resources.leftLaunchMotor;
import static ca.mcgill.ecse211.project.Resources.rightLaunchMotor;
import static ca.mcgill.ecse211.project.Resources.MAXIMUM_LAUNCH_DISTANCE;
import static ca.mcgill.ecse211.project.Resources.MINIMUM_LAUNCH_DISTANCE;

import lejos.utility.Delay;

/**
 * Class that offers static methods used to operate the robot's launching arm.
 * 
 * @author Yi Heng Liu
 * @author Marie Guertin
 * @author Karl Koerich
 * @author Sofia Dieguez
 * @author Lukas Durand
 */
public class Launcher {
 
  /**
   * Launch ball to specified location set in Resources after turning the robot to face the target. Turns robot back to
   * its heading before launch and backs it up to account for launch recoil.
   * 
   * @param launchPos position of robot at launch
   * @param target target location
   */
  public static void launchSequence(double[] launchPos, int[] target) {
    double ANGLE_BEFORE_LAUNCH = odometer.getT();

    // Turn robot to face target location
    turnTo(target[0] * TILE_SIZE, target[1] * TILE_SIZE);
    int launchSpeed = setLaunchSpeed(launchPos, target);

    reload();
    for (int i = 0; i < NUMBER_OF_LAUNCHES; i++) {
      Delay.msDelay(3000);
      setLaunchAcceleration();
      launch(launchSpeed);

      reload();
    }

    Sound.beep();
    stop();
    Main.sleepFor(100);

    leftLaunchMotor.flt();
    rightLaunchMotor.flt();

    Main.sleepFor(100);
    turnTo(ANGLE_BEFORE_LAUNCH);
    Main.sleepFor(50);
    travelBack(OFFSET * 2);
    Main.sleepFor(50);
  }

  /**
   * Simple ball launch for testing purposes which launches a given number of time from Resources
   * 
   */
  public static void launchSequence() {

    for (int i = 0; i < NUMBER_OF_LAUNCHES; i++) {
      Delay.msDelay(3000);
      setLaunchAcceleration();
      launch(240);
      reload();
    }

    Sound.beep();
    stop();
  }

  /**
   * Calculates and returns the speed of the launch motors required to shoot from launchTile to target
   * 
   * @param launchTile Robot's location (tile coordinates)
   * @param target Target bin's location (tile coordinates)
   * @return launchSpeed Launch motor speed
   */
  private static int setLaunchSpeed(double[] launchTile, int[] target) {

    double a = Math.abs(launchTile[0] * TILE_SIZE - target[0] * TILE_SIZE);
    double b = Math.abs(launchTile[1] * TILE_SIZE - target[1] * TILE_SIZE);

    double x = Math.sqrt(a * a + b * b) + LAUNCH_OFFSET; // launch distance with offset of launching arm from robot

    // Best-fit polynomial equation describing the relation
    // between launch motor speed and distance traveled by a launched ping-pong ball
    int launchSpeed =
        (int) (((3) * (Math.pow(10, -6)) * (Math.pow(x, 3))) - (0.0009 * (Math.pow(x, 2))) + (0.5046 * (x)) + 124.6);

    Main.sleepFor(100);

    return launchSpeed;

  }

  /**
   * Set launch motor accelerations
   */
  private static void setLaunchAcceleration() {
    leftLaunchMotor.setAcceleration(4000);
    rightLaunchMotor.setAcceleration(4000);
  }

  /**
   * Launches ball with specified velocity
   * 
   * @param speed: Motor speed passed to launch motors
   */
  private static void launch(int speed) {
    leftLaunchMotor.setSpeed(speed); //
    rightLaunchMotor.setSpeed(speed);
    leftLaunchMotor.rotate(-110, true);
    rightLaunchMotor.rotate(-110, false);
  }

  /**
   * Reloads ball by reseting arm to initial position
   */
  private static void reload() {
    leftLaunchMotor.flt();
    rightLaunchMotor.flt();
    Main.sleepFor(500);
    leftLaunchMotor.setSpeed(200);
    rightLaunchMotor.setSpeed(200);
    leftLaunchMotor.rotate(30, true);
    rightLaunchMotor.rotate(30, false);
    Main.sleepFor(100);
    leftLaunchMotor.flt();
    rightLaunchMotor.flt();
    Main.sleepFor(500);
    leftLaunchMotor.setSpeed(250);
    rightLaunchMotor.setSpeed(250);
    leftLaunchMotor.rotate(110, true);
    rightLaunchMotor.rotate(110, false);
  }

  /**
   * Stops the launcher
   */
  private static void stop() {
    leftLaunchMotor.setSpeed(0);
    rightLaunchMotor.setSpeed(0);
    leftLaunchMotor.stop();
    rightLaunchMotor.stop();
  }

  /**
   * Lift the launching arm back up after lowering it to go through tunnel
   */
  public static void releaseLauncher() {
    synchronized (leftLaunchMotor) {
      synchronized (rightLaunchMotor) {
        leftLaunchMotor.flt();
        rightLaunchMotor.flt();
      }
    }
  }

  /**
   * This method calculates whether we can launch from the current distance to the launch point. It returns true if the
   * current distance from the launch point is within the range from which we can shoot.
   * 
   * @param launchPos robot's position in tile coordinates
   * @param target target location in tile coordinates
   * @return canLaunch true if we can launch from current distance to launch point
   */
  public static boolean canLaunch(double[] launchPos, double[] target) {

    boolean canLaunch = false;

    double dx = target[0] - launchPos[0];
    double dy = target[1] - launchPos[1];

    double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    if (distance >= MINIMUM_LAUNCH_DISTANCE && distance <= MAXIMUM_LAUNCH_DISTANCE) {
      canLaunch = true;
    }

    return canLaunch;
  }


}
