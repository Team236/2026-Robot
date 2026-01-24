package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPivotTowardHub extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;
  private PIDController pidController;
  private double HUBX;
  private double HUBY;

  public AutoPivotTowardHub(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    //this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

    // need help understanding what k (difference) and d (rate) should be
    pidController = new PIDController(
        0.0,
        0.0,
        0.0);

    // used for shortest route (angle measurement that wraps around)
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    // tolerance is to prevent gittering (this will need to be tuned)
    pidController.setTolerance(Math.toRadians(1.0));
    pidController.setSetpoint(0.0);
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();

    // alliance hub selection
    // this needs to be reviewd to make sure code is aceptable and intergratable
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
    // took from swerve
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
      //double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

      // math concepts at https://tinyurl.com/mvjft42z
      Pose2d currentPose = s_Swerve.getPose();

      // needs to be in radians for math.atan2()
      double dx = HUBX - currentPose.getX();
      double dy = HUBY - currentPose.getY();
      double targetAngle = Math.atan2(dy, dx);

      double pidOutput = pidController.calculate(currentPose.getRotation().getRadians(), targetAngle);

      double newRotation = MathUtil.clamp(pidOutput, -1.0, 1.0);

      // if driver wants to turn while holding and it gets springed back
      // use: newRotationVal = (newRotation * .75) + (rotaionVal * .25)

    //   if (pidController.atSetpoint()) {
    //     newRotation = 0;
    // }

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        newRotation * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(), // this might need to be set only true
        true
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}