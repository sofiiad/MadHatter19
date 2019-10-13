package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import static ca.mcgill.ecse211.lab4.Resources.*;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import ca.mcgill.ecse211.lab4.Display;

/**
 * The main driver class for the Localization lab.
 */
public class Main {

  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {

    int buttonChoice;
    buttonChoice = chooseRisingOrFallingEdge();

    // INFO ON RISING AND FALLING EDGE:

    // Rising edge: the point at which the measured distance rises above d
    // The robot starts facing a wall. It can detect a rising edge, switch
    // directions, then detect another rising edge or continue in same direction
    // and detect a falling edge.

    // Falling edge: the point at which the measured distance falls below d
    // Start facing away from the walls. It can detect a falling edge, switch
    // directions, then detect another falling edge or continue in same direction
    // and detect a rising edge.
    // Only when the distance falls below a value d - noise margin is the
    // falling edge detected


    LCD.clear();		

    // Start the usPoller thread
    new Thread(usPoller).start();
    // Start the odometer thread
    new Thread(odometer).start();


    if (buttonChoice == Button.ID_LEFT) { // Rising Edge option chosen
      usLocalizer.risingEdge();

    } else if (buttonChoice == Button.ID_RIGHT) {// Falling edge option chosen
      usLocalizer.fallingEdge();
    } else {
    }

    // Wait for button press to start navigation to (1,1)
    Button.waitForAnyPress();
    lsLocalizer.lightLocalization();

    // Wait here until button pressed to terminate localization program
    Button.waitForAnyPress();

    System.exit(0);
  }

  /**
   * Asks the user to choose between rising or falling edge
   * 
   * @return the user choice
   */
  private static int chooseRisingOrFallingEdge() {

    int buttonChoice;

    Display.showText("< Left | Right >", 
        " Rising| Falling", 
        " Edge  | Edge   ", 
        "       |        ",
        "       |        ");

    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }

  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration
   *            sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

  /**
   * Shows error message and exits program.
   */
  public static void showErrorAndExit(String errorMessage) {
    LCD.clear();
    System.err.println(errorMessage);

    // Sleep for 2 seconds so user can read error message
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
    }

    System.exit(-1);
  }

}
