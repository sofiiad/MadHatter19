package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.TRACK;
import static ca.mcgill.ecse211.lab4.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.lab4.Resources.leftMotor;
import static ca.mcgill.ecse211.lab4.Resources.rightMotor;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import ca.mcgill.ecse211.lab4.Odometer;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 *
 */

public class Odometer implements Runnable {

  /**
   * The x-axis position in cm.
   */
  private double x;

  /**
   * The y-axis position in cm.
   */
  private double y; // y-axis position

  /**
   * The orientation in degrees.
   */
  private double theta; // Head angle

  /**
   * The (x, y, theta) position as an array
   */
  private static double[] position;

  /**
   * Last tacho counts for left and right motor
   */
  public static int lastTachoL;
  public static int lastTachoR;

  // Thread control tools
  /**
   * Fair lock for concurrent writing
   */
  private static Lock lock = new ReentrantLock(true);

  /**
   * Indicates if a thread is trying to reset any position parameters
   */
  private volatile boolean isResetting = false;

  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();

  private static Odometer odo; // Returned as singleton

  // Motor-related variables
  private static int leftMotorTachoCount = 0;
  private static int rightMotorTachoCount = 0;

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;

  /**
   * This is the default constructor of this class. It initiates all motors and
   * variables once.It cannot be accessed externally.
   */
  private Odometer() {
    setXYT(0, 0, 0);
  }

  /**
   * Returns the Odometer Object. Use this method to obtain an instance of
   * Odometer.
   * 
   * @return the Odometer Object
   */
  public synchronized static Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }

    return odo;
  }

  /**
   * This method is where the logic for the odometer will run.
   */
  public void run() {
    long updateStart, updateEnd;
    double dX, dY, dTheta, distL, distR, dDistance;

    leftMotor.resetTachoCount(); // Clear tacho counts
    rightMotor.resetTachoCount();

    lastTachoL = leftMotor.getTachoCount(); // Initialize tacho count
    lastTachoR = rightMotor.getTachoCount();

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      // Calculate new robot position based on tachometer counts

      position = getXYT(); // get the coordinates

      distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastTachoL) / 180; // compute wheel displacement
      distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastTachoR) / 180;
      dDistance = 0.5 * (distL + distR);// compute robot displacement

      dTheta = (distL - distR) / (2 * TRACK); // Compute change in direction
      dTheta = Math.toDegrees(dTheta); // Conversion to degrees
      theta += dTheta; // Updated theta

      // Use of trigonometry to convert polar into cartesian coordinats
      dX = dDistance * Math.sin(Math.toRadians(theta)); // Compute delta x : component of displacement
      dY = dDistance * Math.cos(Math.toRadians(theta)); // Compute delta y component of displacement

      // Update tacho counts for next iteration
      lastTachoL = leftMotorTachoCount;
      lastTachoR = rightMotorTachoCount;

      // Update odometer values with new calculated values, eg
      odo.update(dX, dY, dTheta);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  // IT IS NOT NECESSARY TO MODIFY ANYTHING BELOW THIS LINE

  /**
   * Returns the Odometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the odoData
   * array. {@code odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;}
   * 
   * @param position
   *            the array to store the odometer data
   * @return the odometer data.
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively.
   * Useful for odometry.
   * 
   * @param dx
   * @param dy
   * @param dtheta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates within 360 degrees
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }

  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x
   *            the value of x
   * @param y
   *            the value of y
   * @param theta
   *            the value of theta in degrees
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x
   *            the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y
   *            the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta
   *            the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Returns the position array. Use this method to obtain the position of the
   * robot.
   * 
   * @return position;
   */
  public static double[] getPosition() {
    return position;
  }

  /**
   * Returns the value of x.
   * 
   * @return the value of x.
   */
  public double getX() {
    synchronized (this) {
      return x;
    }
  }

  /**
   * Returns the value of y.
   * 
   * @return the value of y.
   */
  public double getY() {
    synchronized (this) {
      return y;
    }
  }

  /**
   * Returns the value of theta.
   * 
   * @return the value of theta.
   */
  public double getTheta() {
    synchronized (this) {
      return theta;
    }
  }



}
