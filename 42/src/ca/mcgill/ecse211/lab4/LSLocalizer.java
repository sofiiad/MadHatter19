package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import static ca.mcgill.ecse211.lab4.Resources.colorSensor;
import static ca.mcgill.ecse211.lab4.Resources.*;
import static java.lang.Math.*;

public class LSLocalizer {

  //********************************
  //        VARIABLES
  //********************************

  private static float[] colorSample; // Store data retrieved from light sensor
  private float prevColor = 0; // Initializing variable which will store previous color reading
  private static int NB_OF_LINES = 0; //Initializing line count
  private double[] lineAngles = new double[4]; //Array which will store headings when sensor detects a line
  public static boolean sensorInitialized = false;//Initializing variable which accounts for 
  //the light sensor being initializes


  /**
   * Method to make the robot travel to the destination located at the waypoint (1,1)  
   * */
  public void travelToDestination() {
    //Turn to 45 degrees to start light sensor localization
    Navigation.turnTo(45);

    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.setSpeed(ROTATE_SPEED);
    colorSensor.fetchSample(colorSample, 0);

    //EV3 starts moving until the light sensor detects a line
    do {
      rightMotor.forward();
      leftMotor.forward();
    } while (!lineSeen());

    Sound.beep(); //Trigger sound to indicate when a line has been detected

    //Stop EV3 
    Navigation.stopMotors();

    // Make the robot go a bit backwards since the light sensor is placed in the back (offset)
    Navigation.travelBack(-LS_SENSOR_WHEELS_OFFSET);

  }//End of travelToDestination method

  //************************************
  //        LIGHT LOCALIZATION
  //************************************
  public void lightLocalization () {

    //Declaring variables that will be used in the trigonometric calculations 
    double x;
    double theta_X;
    double y;
    double theta_Y;

    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.setSpeed(ROTATE_SPEED);

    NB_OF_LINES = 0; //Initialize nb of lines detected
    initializeSensor();//Initialize sensor

    travelToDestination();//Travel to (1,1)

    int index = 0; //Initializing counter to store headings

    do {
      //Make the robot spin around its center of rotation
      leftMotor.forward();
      rightMotor.backward();

      if (lineSeen()) {
        Sound.beep();//Trigger sound to indicate a line has been detected
        lineAngles[index] = odometer.getXYT()[2];//Store the angle at which the line was detected
        //lineAngles[0] : first x-axis line crossed
        //lineAngles[1]: first y-axis line crossed
        //lineAngles[2]: second x-axis line crossed
        //lineAngles[3]: second y-axis line crossed
        index++;
      } 
    } while (index < 4);//Do until 4 lines have been crossed


    //Stop the EV3 when 4 lines have been detected
    Navigation.stopMotors();

    /*Compute the angles that subtends the arc connecting the intersections of 
     * the light sensor's path with the y axis called theta_Y as well as for the
     * x axis called theta_X 
     */
    theta_Y= lineAngles[3] - lineAngles[1];
    x = (-(LS_SENSOR_WHEELS_OFFSET)) * Math.cos(Math.toRadians(theta_Y/2));
    theta_X = lineAngles[2] - lineAngles[0];
    y = (-(LS_SENSOR_WHEELS_OFFSET)) * Math.cos(Math.toRadians(theta_X/2));

    //Set correct values in odometer
    odometer.setXYT(x, y, odometer.getXYT()[2]);

    //Travel to the (1,1) waypoint now that the correction has been applied
    //Navigation.travelTo(1.0,1.0);
    rightMotor.setSpeed(ROTATE_SPEED/2);
    leftMotor.setSpeed(ROTATE_SPEED/2);

    //Turn robot to finish in a 0 degree angle
    Navigation.turnTo(0);

    //    //Rotate the robot for it to finish at a 0 degree orientation
    //    if(odometer.getXYT()[2] > 10.0 && odometer.getXYT()[2] < 350) {
    //      //      rightMotor.rotate((int) -(rotationAngle(-odometer.getXYT()[2])));
    //      //      leftMotor.rotate((int) rotationAngle(-odometer.getXYT()[2]));
    //      rightMotor.rotate((int) -(rotationAngle(0)));
    //      leftMotor.rotate((int) rotationAngle(0));
    //    }

    //Stop robot
    Navigation.stopMotors();
    Display.updateDisplay();

  }//End of lightLocalization method

  //*********************************
  //        OTHER METHODS
  //*********************************

  /** 
   * Method which rotates the robot's wheels to attain its desired orientation
   * @param degree
   *  */
  public static void turnToAngle(double degree) {
    leftMotor.rotate((int) rotationAngle(degree), true);
    rightMotor.rotate(-((int) rotationAngle(degree)), false);
  }//End of turnToAngle method

  /**
  Method which computes the most efficient turn to attain the desired angle
  @param initAngle: initial angle of robot
  @param desiredAngle: desired angle of robot
   */
  public static double bestTurn(double initAngle, double desiredAngle) {
    double result = (desiredAngle - initAngle + 540) % 360 - 180;
    return result;
  }

  /**
   * This method tells us if any lines were detected
   * */
  public static boolean lineSeen() {
    if (!sensorInitialized) {
      initializeSensor();
    }
    float[] curColorSample;
    curColorSample = new float[1];
    colorSensor.getMode("Red"); // Set sensor to only use red light
    colorSensor.fetchSample(curColorSample, 0);

    //Differential filter
    if (curColorSample[0] < 0.7 * colorSample[0]) {
      NB_OF_LINES++;
      return true;
    } else {
      return false;
    }
  }
  /**
   * This method is used to initialize the light sensor */
  public static void initializeSensor() {
    colorSample = new float[1];
    colorSensor.setCurrentMode("Red"); // Set sensor to only use red light
    colorSensor.fetchSample(colorSample, 0);
    sensorInitialized = true;
  }//End of initializeSensor() method

  /*
   * Method used to find how many wheel rotations are needed for the robot to travel its desired distance
   * @param distance
   * **/  
  private static double rotationDist(double distance) {
    double result = ((180.0 * distance)/(Math.PI * WHEEL_RAD));
    return result;
  }//End of convertAngle method

  /*
   * Method used to find how many wheel rotations are needed for the robot to arrive to its desired orientation
   * @param theta: orientation desired 
   * **/  
  private static double rotationAngle(double theta) {
    double distance = (Math.PI * TRACK * (theta/360.0));
    double result = ((180.0 * distance)/(Math.PI * WHEEL_RAD));
    return result;
  }//End of convertAngle method


}//End of LSLocalizer class