package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Field2d field;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    Constants.Targeting.hoodAngleMap.get(0.0);
    Constants.Targeting.rpmMap.get(0.0);
    Constants.Targeting.preFeederMap.get(0.0);
    Constants.Targeting.timeMap.get(0.0);
    Constants.Targeting.nuetralMap.get(0.0);

    DashboardUtil.putAutoToField();
    Constants.ChangableBinConstants.pushToDashboard();

    UsbCamera usbCamera0;
    try {
      usbCamera0 = CameraServer.startAutomaticCapture(0);
    } catch (Exception e) {
      SmartDashboard.putString("camera capture failed", "failed");
    }

    TrajectoryConfig config = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5805, "limelight.local", 5805);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Shift Time", DashboardUtil.getShiftTime());
    SmartDashboard.putString("Current Shift", DashboardUtil.getCurrentShift());
    Constants.ChangableBinConstants.pullFromDashboard();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle-set").setNumber(150);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle-set").setNumber(0);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle-set").setNumber(0);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
