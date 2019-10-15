package ca.mcgill.ecse211.lab1;

import lejos.hardware.Button;

import static ca.mcgill.ecse211.lab1.Resources.*;

/**
 * Main class of the program.
 */
public class Main {
  
  /**
   * The US controller selected by the user (bang-bang or P-type).
   */
  public static UltrasonicController selectedController;

  /**
   * Main entry point - instantiate objects used and set up sensor
   * @param args
   */
  public static void main(String[] args) {
    // Set up the display on the EV3 screen and wait for a button press. 
    // The button ID (option) determines what type of control to use
    Printer.printMainMenu();
    int option = Button.waitForAnyPress();

    if (option == Button.ID_LEFT) {
      selectedController = new BangBangController();
    } else if (option == Button.ID_RIGHT) {
      selectedController = new PController();
    } else {
      showErrorAndExit("Error - invalid button!");
    }

    // Start the poller and printer threads
    new Thread(new UltrasonicPoller()).start();
    new Thread(new Printer()).start();

    // Wait here until button pressed to terminate wall follower
    Button.waitForAnyPress();
    System.exit(0);
  }

  /**
   * Shows error and exits program.
   */
  public static void showErrorAndExit(String errorMessage) {
    TEXT_LCD.clear();
    System.err.println(errorMessage);
    
    // Sleep for 2 seconds so user can read error message
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
    }
    
    System.exit(-1);
  }
  
}
