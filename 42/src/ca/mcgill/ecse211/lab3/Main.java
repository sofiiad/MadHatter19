package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.Sound;

import static ca.mcgill.ecse211.lab3.Resources.*;
import ca.mcgill.ecse211.lab3.Display;

/**
 * The main driver class for the navigation lab.
 */
public class Main {

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {


		int buttonChoice;		
		buttonChoice = chooseObstacleAvoidanceOrNot();
		
		boolean avoidObstacles;
		if (buttonChoice == Button.ID_LEFT) {
			avoidObstacles = false;
		} else {
			avoidObstacles = true;
			new Thread(usPoller).start();
			new Thread(obstacleAvoidance).start();
		}
		
		new Thread(odometer).start();

		completeCourse(avoidObstacles);

		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		} // do nothing

		System.exit(0);
	}

	/**
	 * Completes a course.
	 */
	private static void completeCourse(boolean avoid) {

		odometer.setX(TILE_SIZE);
		odometer.setY(TILE_SIZE);

		for (int[] point : map_4) {

			Navigation.travelTo(point[0] * TILE_SIZE, point[1] * TILE_SIZE, avoid);
			while (ObstacleAvoidance.traveling) {
				Display.updateDisplay();
				Main.sleepFor(500);
			}
			Display.updateDisplay();
			Sound.beep();
		}
		Sound.beep();
		Sound.beep();
		Sound.beep();
	}

	/**
	 * Asks the user whether the robot needs to avoid obstacles or not.
	 *
	 * @return the user choice
	 */
	private static int chooseObstacleAvoidanceOrNot() {
		int buttonChoice;
		Display.showText("< Left | Right >",
                		 " No    | with   ",
                		 " obstc | obstc  ",
                		 " les   | les    ",
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

}
