package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** The absolute entry point of the robot software application. */
public final class Main {

  /** Private constructor to prevent instantiation of the Main class (utility container). */
  private Main() {
  }

  /**
   * The standard main method required by all Java applications.
   * This is the literal first line of code executed on the RoboRIO when the code deploy completes.
   * * @param args Arguments passed from the operating system command line (unused in WPILib).
   */

  public static void main(String... args) {
    
    // Starts the WPILib background processes and instantiates our custom master control loop.
    // "Robot::new" is a Java method reference that tells WPILib how to build a new instance
    // of our custom Robot class.
    RobotBase.startRobot(Robot::new);
  }
}
