package ca.mcgill.ecse211.lab3;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

/**
 * Class used for logging
 */
public class Log {

  static PrintStream writer = System.out;

  public static enum Sender {
    odometer, Navigator, usSensor, avoidance
  }

  static boolean printOdometer;
  static boolean printNavigator;
  static boolean printUsSensor;
  static boolean printAvoidance;

  public static void log(Sender sender, String message) {
    long timestamp = System.currentTimeMillis() % 100000;

    if (sender == Sender.Navigator && printNavigator) {
      writer.println("NAV::" + timestamp + ": " + message);
    }
    if (sender == Sender.odometer && printOdometer) {
      writer.println("ODO::" + timestamp + ": " + message);
    }
    if (sender == Sender.usSensor && printUsSensor) {
      writer.println("US::" + timestamp + ": " + message);
    }
    if (sender == Sender.avoidance && printAvoidance) {
      writer.println("OA::" + timestamp + ": " + message);
    }

  }

  public static void setLogging(boolean nav, boolean odo, boolean us, boolean avoid) {
    printNavigator = nav;
    printOdometer = odo;
    printUsSensor = us;
    printAvoidance = avoid;
  }

  public static void setLogWriter(String filename) throws FileNotFoundException {
    writer = new PrintStream(new File(filename));
  }

}
