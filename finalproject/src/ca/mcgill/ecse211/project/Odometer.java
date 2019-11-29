package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 * 
 * @author Yi Heng Liu
 * @author Sofia Dieguez
 * @author Marie Guertin
 * @author Lukas Durand
 * @author Karl Koerich
 */

public class Odometer implements Runnable {

  /**
   * The x-axis position in cm.
   */
  private volatile double x;

  /**
   * The y-axis position in cm.
   */
  private volatile double y;

  /**
   * The orientation in degrees.
   */
  private volatile double theta;

  /**
   * The (x, y, theta) position as an array
   */
  private double[] position;

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

  /**
   * Returned as singleton
   */
  private static Odometer odo;

  /**
   * Position-related variables
   */
  private double distL, distR, dDist, dTheta, dX, dY;

  /**
   * Motor-related variables
   */
  private static int leftMotorTachoCount = 0, rightMotorTachoCount = 0, prevLeftMotorTachoCount,
      prevRightMotorTachoCount;

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;


  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It cannot be accessed
   * externally.
   */
  Odometer() {
    setXYT(0, 0, 0);
  }

  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
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
    prevLeftMotorTachoCount = 0;
    prevRightMotorTachoCount = 0;

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      position = getXYT();

      // Calculate distance traveled by each wheel based on tachometer counts
      distL = Math.PI * (WHEEL_RAD) * (leftMotorTachoCount - prevLeftMotorTachoCount) / 180;
      distR = Math.PI * (WHEEL_RAD) * (rightMotorTachoCount - prevRightMotorTachoCount) / 180;

      // Store previous tachometer counts
      prevLeftMotorTachoCount = leftMotorTachoCount;
      prevRightMotorTachoCount = rightMotorTachoCount;

      // Calculate new angle based on distance traveled by each wheel
      dDist = 0.5 * (distL + distR);
      dTheta = (distL - distR) / TRACK;
      dTheta = Math.toDegrees(dTheta);
      position[2] += dTheta;

      // Separate displacement into X and Y vectors based on angle
      dX = dDist * Math.sin(Math.toRadians(position[2]));
      dY = dDist * Math.cos(Math.toRadians(position[2]));

      // Update odometer values with new calculated values
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

  /**
   * The methods are as provided by the DPM professors
   * 
   * @author Rodrigo Silva
   * @author Dirk Dubois
   * @author Derek Yu
   * @author Karim El-Baba
   * @author Michael Smith
   * @author Younes Boubekeur
   */

  /**
   * Returns the Odometer data.
   * 
   * Writes the current position and orientation of the robot onto the odoData array. {@code odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;}
   * 
   * position the array to store the odometer data
   * @return position the odometer data.
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
   * Returns the x value of the Odometer data.
   * 
   * Writes the current x position of the robot
   * 
   * @return xPos the value of x
   */
  public double getX() {
    double xPos = 0;

    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      xPos = x;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return xPos;
  }

  /**
   * Returns the y value of the Odometer data.
   * 
   * Writes the current y position of the robot
   * 
   * @return yPos the value of y
   */
  public double getY() {
    double yPos = 0;

    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      yPos = y;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return yPos;
  }

  /**
   * Returns the theta value of the Odometer data.
   * 
   * Writes the current theta orientation of the robot
   * 
   * @return tPos the value of theta
   */
  public double getT() {
    double tPos = 0;

    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      tPos = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return tPos;
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for odometry.
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
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
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
   * @param x the value of x
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
   * @param y the value of y
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
   * @param theta the value of theta
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
}
