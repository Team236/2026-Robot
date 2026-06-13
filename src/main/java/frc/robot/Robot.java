package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM (Virtual Machine) on the RoboRIO is configured to automatically run
 * this class.
 * This is the master control loop for the robot. It handles the transition
 * between
 * different states (Autonomous, Teleop, Disabled, Test) and ensures the
 * CommandScheduler
 * runs continuously to process subsystem logic.
 */

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public Field2d field;

  /**
   * This function is run exactly once when the RoboRIO first boots up.
   * Use this for hardware initialization, building autonomous choosers, and
   * "warming up" complex code paths so they don't lag the first time they are
   * called.
   */

  @Override
  public void robotInit() {
    // Instantiates our subsystems and button bindings.
    m_robotContainer = new RobotContainer();

    // WARMUPS: Java lazily loads classes. By running these commands and map lookups
    // during boot, we force the RoboRIO to process them now. If we wait until
    // autonomous
    // starts to load these, the robot will momentarily freeze (loop overrun) right
    // as the match begins.
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    Constants.Targeting.hoodAngleMap.get(0.0);
    Constants.Targeting.rpmMap.get(0.0);
    Constants.Targeting.preFeederMap.get(0.0);
    Constants.Targeting.timeMap.get(0.0);
    Constants.Targeting.nuetralMap.get(0.0);

    DashboardUtil.putAutoToField();
  }

  /**
   * This function is called every 20 milliseconds (50 times a second), no matter
   * what mode
   * the robot is in.
   * Because it runs so frequently, only put lightweight updates here, like
   * pushing diagnostic
   * data to the SmartDashboard. Putting heavy mathematical logic here will cause
   * loop overruns.
   */

  @Override
  public void robotPeriodic() {
    // This single line tells every active command to execute its next step,
    // and every subsystem to run its periodic() method.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Shift Time", DashboardUtil.getShiftTime());
    SmartDashboard.putString("Current Shift", DashboardUtil.getCurrentShift());

    // Continuously polls the dashboard for live bin tuning updates.
    Constants.ChangableBinConstants.pullFromDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * Used to force mechanisms into a safe state.
   */

  @Override
  public void disabledInit() {
    // Stops all active mechanical actions for safety.
    CommandScheduler.getInstance().cancelAll();

    // Puts the Limelight into a low-bandwidth or low-intensity state
    // while sitting on the field waiting for the match to start.
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle-set").setNumber(150);
  }

  /**
   * This function is called periodically (every 20ms) while the robot is
   * disabled.
   */

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function runs once the moment the autonomous period begins.
   * It pulls the selected routine from ({@link RobotContainer}) and schedules it.
   */

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Wakes up the Limelight for active target tracking (throttle set 0).
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle-set").setNumber(0);
  }

  /** This function is called periodically (every 20ms) during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Example of where odometry updates using vision features like MegaTag2 would
    // be called.
  }

  /**
   * This function runs once the moment the teleoperated (driver-controlled)
   * period begins.
   */

  @Override
  public void teleopInit() {
    // Ensures that any leftover autonomous routines are killed before giving
    // joysticks control to the drivers.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Wakes up the Limelight for active target tracking during teleop.
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle-set").setNumber(0);
  }

  /** This function is called periodically (every 20ms) during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once when test mode begins. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode to ensure a clean slate.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically (every 20ms) during test mode. */
  @Override
  public void testPeriodic() {
  }
}