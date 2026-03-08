package frc.robot.commands.Targeting;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TranslationSubsystem;
import frc.robot.subsystems.StrafeSubsystem;
import frc.robot.subsystems.RobotCentricSubsystem;
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
  private boolean isRedAlliance;
  private double newRotation;

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
      isRedAlliance = true;
    } else {
      isRedAlliance = false;
    }
  }

  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

    if (isRedAlliance) {
      if (s_Swerve.inNeutralZone(isRedAlliance)) {
        
      } else {
        
      }
    } else {
      if (s_Swerve.inNeutralZone(isRedAlliance)) {
        
      } else {
        
      }
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
