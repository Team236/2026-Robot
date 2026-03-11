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

public class AutoPivotTowardHub extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;
  private double HUBX;
  private double HUBY;
  private double newRotation;

  public AutoPivotTowardHub(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
    // USING SWERVE FOR TAKING OVER ROTATION
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;
    addRequirements(s_Swerve);
  }

  private double curveDrive(double input) {
  return 0.5 * Math.pow(input, 3) + 0.5 * input;
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
  }

  @Override
  public void execute() {
    // X AND Y VALUES FOR DRIVING
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

    double angleDelta = Units.radiansToDegrees(s_Swerve.getAngleOfHub(HUBX, HUBY)) - s_Swerve.getHeading().getDegrees();
    double angleDeltaMod = MathUtil.inputModulus(angleDelta, -180, 180);
    // SmartDashboard.putNumber("angle delta", angleDelta);
    // SmartDashboard.putNumber("angle delta mod", angleDeltaMod);
    // SmartDashboard.putNumber("robot angle", s_Swerve.getHeading().getDegrees());
    // SmartDashboard.putNumber("hub angle", Units.radiansToDegrees(s_Swerve.getAngleOfHub(HUBX, HUBY)));
    // MATH IS AT https://tinyurl.com/mvjft42z
    newRotation = this.s_Swerve.calculateTargetingPID(HUBX, HUBY);
    // if current angle is within tolerance of target, don't feed any rotation

    // if (Math.abs(angleDeltaMod) > Constants.Targeting.AUTO_ROTATE_TOLERANCE) {
    //   newRotation += Constants.Targeting.AUTO_ROTATE_FEEDFORWARD * Math.signum(angleDeltaMod);
    // }

    // DRIVING COMMAND THAT JUST INPUTS COMPUTERS ROTATION
    s_Swerve.drive(
        new Translation2d(curveDrive(translationVal), curveDrive(strafeVal)).times(Constants.Swerve.maxSpeed),
        MathUtil.clamp(newRotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity),
        !robotCentricSup.getAsBoolean(),
        true
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}