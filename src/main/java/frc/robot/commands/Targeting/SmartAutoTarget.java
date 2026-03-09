package frc.robot.commands.Targeting;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class SmartAutoTarget extends Command {
/** Creates a new SmartAutoTarget. */

private Swerve s_Swerve;
private DoubleSupplier translationSup;
private DoubleSupplier strafeSup;
private BooleanSupplier robotCentricSup;
private double newRotation;
private double PIDRotation;
private double HUBX;
private double HUBY;

public SmartAutoTarget(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
  // USING SWERVE FOR TAKING OVER ROTATION
  this.s_Swerve = s_Swerve;
  this.translationSup = translationSup;
  this.strafeSup = strafeSup;
  this.robotCentricSup = robotCentricSup;


  addRequirements(s_Swerve);
}

// CURVE FOR DRIVE (CONSISTENT WITH NORMAL DRIVING)
private double curveDrive(double input) {
  return 0.5 * Math.pow(input, 3) + 0.5 * input;
}

@Override
public void initialize() {
  var alliance = DriverStation.getAlliance();

  // ALLIANCE SELECTION FROM DRIVER STATION
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
   double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
   double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

   PIDRotation = this.s_Swerve.getAllianceWallHeading(HUBX, HUBY);

   // DRIVE COMMAND WITH THE NEW INPUTS FOR ROTATION
   s_Swerve.drive(
     new Translation2d(curveDrive(translationVal), curveDrive(strafeVal)).times(Constants.Swerve.maxSpeed),
     MathUtil.clamp(PIDRotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity),
     !robotCentricSup.getAsBoolean(),
     true
  );
}

@Override
public void end(boolean interrupted) {
}

@Override
public boolean isFinished() {
  return false;
}
}