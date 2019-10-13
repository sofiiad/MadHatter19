package ca.mcgill.ecse211.lab2;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab2.Resources.*;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class SquareDriver {

	/**
	 * Drives the robot in a square of size 3x3 Tiles. It is to be run in parallel
	 * with the odometer and odometer correction classes to allow testing their
	 * functionality.
	 * 
	 * @param correction Flag for using correction or not
	 * 
	 */
	public static void drive(final boolean correction) {
		// spawn a new Thread to avoid this method blocking
		(new Thread() {
			public void run() {

				// reset the motors
				leftMotor.stop();
				rightMotor.stop();
				leftMotor.setAcceleration(ACCELERATION);
				rightMotor.setAcceleration(ACCELERATION);

				// Sleep for 2 seconds
				Main.sleepFor(TIMEOUT_PERIOD);

				double[] position;
				int i = 0; 

				while (i != 4) {

					// drive forward 
					leftMotor.setSpeed(FORWARD_SPEED);
					rightMotor.setSpeed(FORWARD_SPEED);
					rightMotor.forward();
					leftMotor.forward();

					// Get the current position of the robot
					if (correction) {
						position = OdometryCorrection.getPosition();
					} else {
						position = Odometer.getPosition();
					}

					if (   i == 0 && position[1] >= NB_OF_LINES * TILE_SIZE + TURN_OFFSET   // Increasing in Y
						|| i == 1 && position[0] >= NB_OF_LINES * TILE_SIZE + TURN_OFFSET	// Increasing in X
						|| i == 2 && position[1] <= 1 * TILE_SIZE - TURN_OFFSET	   			// Decreasing in Y 
						|| i == 3 && position[0] <= 1 * TILE_SIZE - TURN_OFFSET) { 			// Decreasing in X

						i++; 
						// Stop robot
						leftMotor.stop(true);
						rightMotor.stop();

						// turn 90 degrees clockwise
						leftMotor.setSpeed(ROTATE_SPEED);
						rightMotor.setSpeed(ROTATE_SPEED);
						leftMotor.rotate(convertAngle(90.0), true);
						rightMotor.rotate(-convertAngle(90.0), false);
					}

				}
			}
		}).start();
	}

	/**
	 * Converts input distance to the total rotation of each wheel needed to cover
	 * that distance.
	 * 
	 * @param distance
	 * @return the wheel rotations necessary to cover the distance
	 */
	public static int convertDistance(double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

	/**
	 * Converts input angle to the total rotation of each wheel needed to rotate the
	 * robot by that angle.
	 * 
	 * @param angle
	 * @return the wheel rotations necessary to rotate the robot by the angle
	 */
	public static int convertAngle(double angle) {
		return convertDistance(Math.PI * TRACK * angle / 360.0);
	}
}
