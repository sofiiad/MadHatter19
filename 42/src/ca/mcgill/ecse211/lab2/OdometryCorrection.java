package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;

import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {

	private static final long CORRECTION_PERIOD = 10;

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
	 * The (x, y, theta) position as an array
	 */
	private static double[] position;

	/**
	 * THe referential color sample stored as an array
	 */
	private static float[] refColorSample;

	private static boolean sensorInitialized = false;

	/**
	 * This method is used to initialize the sensor
	 */
	public static void initializeSensor() {
		refColorSample = new float[1];
		colorSensor.setCurrentMode("Red"); // Set sensor to only use red light
		colorSensor.fetchSample(refColorSample, 0);
		sensorInitialized = true;
	}

	/*
	 * Here is where the odometer correction code should be run.
	 */
	public void run() {
		long correctionStart, correctionEnd; // time elapsed
		int nbOfLinesSeen;
		double x, y, theta;

		position = new double[3];

		nbOfLinesSeen = 0; // nb of lines detected

		initializeSensor(); // initializing sensor

		while (true) {
			correctionStart = System.currentTimeMillis();

			position = odometer.getXYT();
			x = position[0];
			y = position[1];
			theta = position[2];

			// Trigger correction when robot crosses a black line
			if (lineSeen()) {

				nbOfLinesSeen++;

				// Calculate new (accurate) robot position

				// Correction in Y
				if (Math.abs(Math.cos(Math.toRadians(theta))) > 0.5) {

					if (Math.cos(Math.toRadians(theta)) < 0) { // Negative Y direction
						Sound.beep();
						y = TILE_SIZE * (NB_OF_LINES - nbOfLinesSeen + 1);

					} else { // Positive Y direction
						Sound.beep();
						y = TILE_SIZE * (nbOfLinesSeen) - SENSOR_WHEELS_OFFSET;
					}
				}

				// correction in X
				else if (Math.abs(Math.sin(Math.toRadians(theta))) > 0.5) {

					if (Math.sin(Math.toRadians(theta)) < 0) { // Negative X direction
						Sound.beep();
						x = TILE_SIZE * (NB_OF_LINES - nbOfLinesSeen + 1);

					} else { // Positive X direction
						Sound.beep();
						x = TILE_SIZE * (nbOfLinesSeen) - SENSOR_WHEELS_OFFSET;
					}
				}

				// Resetting the number of lines
				nbOfLinesSeen = nbOfLinesSeen % NB_OF_LINES;

				// Update odometer with new calculated (and more accurate) values
				odometer.setXYT(x, y, theta);

			}

			// this ensures the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
			}
		}
	}

	public static boolean lineSeen() {
		if (!sensorInitialized) {
			initializeSensor();
		}
		float[] curColorSample;
		curColorSample = new float[1];
		colorSensor.setCurrentMode("Red"); // Set sensor to only use red light
		colorSensor.fetchSample(curColorSample, 0);

		if (curColorSample[0] < 0.7 * refColorSample[0]) {
			return true;
		} else {
			return false;
		}
	}

}
