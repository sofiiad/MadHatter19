package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.ObstacleAvoidance.State.INIT;
import static ca.mcgill.ecse211.lab3.ObstacleAvoidance.State.TURNING;
import static ca.mcgill.ecse211.lab3.ObstacleAvoidance.State.TRAVELING;
import static ca.mcgill.ecse211.lab3.ObstacleAvoidance.State.EMERGENCY;
import static ca.mcgill.ecse211.lab3.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.lab3.Resources.usPoller;

import lejos.hardware.Sound;

import static ca.mcgill.ecse211.lab3.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.lab3.Resources.leftMotor;
import static ca.mcgill.ecse211.lab3.Resources.rightMotor;



public class ObstacleAvoidance implements Runnable {

	/**
	 * The possible states that the robot could be in.
	 */
	enum State {
		/** The initial state. */
		INIT,
		/** The turning state. */
		TURNING,
		/** The traveling state. */
		TRAVELING,
		/** The emergency state. */
		EMERGENCY
	};

	private static ObstacleAvoidance obsAvoidance; // Returned as a singleton

	/**
	 * True when the robot is traveling
	 */
	public static boolean traveling; // false by default

	/**
	 * The destination x.
	 */
	public static double dest_X;

	/**
	 * The destination y.
	 */
	public static double dest_Y;

	/**
	 * {@code true} when obstacle is avoided.
	 */
	public static boolean safe;

	/**
	 * The sleep time.
	 */
	public static final int SLEEP_TIME = 50;

	/**
	 * The current state of the robot.
	 */
	static State state;

	/**
	 * This method is where the logic for the obstacleAvoidance will run.
	 */
	public void run() {
		
		state = INIT;
		
		while(true) {
			
			if (state == INIT) { 
				if (traveling) { // if traveling is false, no obstacle avoidance
					state = TRAVELING;
				}
				
			} else if (state == TRAVELING) {//Robot is traveling
				
				
				if (Navigation.isDone(dest_X, dest_Y)) {//Reached final destination
					Sound.beep();					
					Navigation.stopMotors();
					state = INIT;
					traveling = false;					
					
				} else {
					Navigation.travelTo(dest_X, dest_Y);//Final destination not reached
				}
				
				
			} else if (state == EMERGENCY) {//Obstacle detected		
				avoidObject();	//call method to avoid obstacle			
			}			
		
			Main.sleepFor(SLEEP_TIME);
		
		}		

	}

	/**
	 * This method implements the logic the robot follows to avoid an obstacle detected by the US
	 * */
	public static void avoidObject() {
		
		boolean safe;

		double distance;
		
		Navigation.stopMotors();
		//double destAngle = Navigation.getDestAngle(dest_X, dest_Y);
		do {
			Navigation.turn(90); // clockwise
			Navigation.moveForward(20);
			Navigation.turn(-90);
			Navigation.moveForward(20);
			Navigation.turnTo(dest_X, dest_Y);
			Navigation.turn(-30);
			distance = usPoller.getDistance();
			safe = distance > 20;
			Navigation.turnTo(dest_X, dest_Y);
			Navigation.stopMotors();
			
			
		} while (!safe);//Do while the robot is not safe
		
		Navigation.setInterrupt(false);//reset interrupt to not interrupted
		
		state = TRAVELING;//set state to traveling
		
		
	}
	
	
	
	/**
	 * Updates the heading.
	 */
	public static void updateTravel() {
		Navigation.travelTo(dest_X, dest_Y, false);
	}

	/**
	 * Returns the ObstacleAvoidance object. Use this method to obtain an instance
	 * of ObstacleAvoidance.
	 * 
	 * @return the ObstacleAvoidance Object.
	 */
	public static ObstacleAvoidance getObstacleAvoidance() {
		if (obsAvoidance == null) {
			obsAvoidance = new ObstacleAvoidance();
		}
		return obsAvoidance;
	}

	/**
	 * Sets emergency state when robot is too close to a wall.
	 */
	public static void checkForObstacle() {
		if (usPoller.getDistance() < 20) {
			//Sound.beep();
			//Sound.beep();
			state = EMERGENCY;
		}
	}

}
