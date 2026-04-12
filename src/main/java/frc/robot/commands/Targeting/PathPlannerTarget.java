package frc.robot.commands.Targeting;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.logging.Handler;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerTarget extends Command {
  private Swerve s_Swerve;
  private double HUBX;
  private double HUBY;
  private double newRotation;
  private double angleDelta;

  public PathPlannerTarget(Swerve s_Swerve) {
    // USING SWERVE FOR TAKING OVER ROTATION
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();

    // ALIGENCE SELECTION FROM DRIVER STATION
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      HUBX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
      HUBY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
    } else {
      HUBX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
      HUBY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
    }

    angleDelta = Units.radiansToDegrees(s_Swerve.getAngleOfHub(HUBX, HUBY)) - s_Swerve.getHeading().getDegrees();
  }

  @Override
  public void execute() {
    angleDelta = Units.radiansToDegrees(s_Swerve.getAngleOfHub(HUBX, HUBY)) - s_Swerve.getHeading().getDegrees();

    newRotation = this.s_Swerve.calculateTargetingPID(HUBX, HUBY);

    s_Swerve.drive(
        new Translation2d(0, 0),
        MathUtil.clamp(newRotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity),
        true,
        true
    );
  }

  @Override
  public boolean isFinished() {
    return Math.abs(angleDelta) < Constants.Targeting.AUTO_ROTATE_TOLERANCE;
  }
}