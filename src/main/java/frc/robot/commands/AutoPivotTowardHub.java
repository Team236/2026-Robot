package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPivotTowardHub extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private PIDController pidController;
  private Pose2d robotFieldPose;
  private double tv;
  private int targetId;
  private double yawError;
  private double newRotation;

  public AutoPivotTowardHub(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
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
  public void execute() {
    // took from swerve
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
      double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

      tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
      targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

    // needs to be chanted for red/blue logic
    if (tv == 1 && targetId >= 2 && targetId <=11 ) {

      // inside the if statement to reduce stress if not seeing tag
      robotFieldPose = LimelightHelpers.getBotPose2d_wpiRed("limelight");

      // gets angle of robot
      double angle1 = robotFieldPose.getRotation().getRadians();

      double angle2 = Constants.Targeting.ID_TO_POSE
        // gets the position for the target it sees
        .get(targetId)
        .getRotation()
        .getRadians();

        yawError = MathUtil.angleModulus(angle2 - angle1);

        newRotation = pidController.calculate(yawError);

    } else if (tv == 1 && targetId >= 18 && targetId <=27) {
      // inside the if statement to reduce stress if not seeing tag
      robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

      // gets angle of robot
      double angle1 = robotFieldPose.getRotation().getRadians();

      double angle2 = Constants.Targeting.ID_TO_POSE
        // gets the position for the target it sees
        .get(targetId)
        .getRotation()
        .getRadians();

        yawError = MathUtil.angleModulus(angle2 - angle1);

        newRotation = pidController.calculate(yawError);
    } else {
      // incase it can't see the apriltag, let driver turn normally
      pidController.reset();
      newRotation = rotationVal;
    }

    // passing in the values for drive
    s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            newRotation * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
  }
  @Override
  public boolean isFinished() {
    return false; // runs while trigger is held
  }
}
