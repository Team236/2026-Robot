package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TranslationSubsystem;
import frc.robot.subsystems.StrafeSubsystem;
import frc.robot.subsystems.RobotCentricSubsystem;

public class SmartAutoTarget extends Command {
  /** Creates a new SmartAutoTarget. */

  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;
  private boolean isRedAlliance;
  private double newRotation;

  public SmartAutoTarget(Swerve s_Swerve, TranslationSubsystem translationSup, StrafeSubsystem strafeSup, RobotCentricSubsystem robotCentricSup ) {
    // USING SWERVE FOR TAKING OVER ROTATION
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;

    addRequirements(s_Swerve);
  }

  // CURVE FOR DRIVE (CONSTISTANT WITH NORMAL DRIVING)
  private double curveDrive(double input) {
    return 0.5 * Math.pow(input, 3) + 0.5 * input;
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();

    // ALIGENCE SELECTION FROM DRIVER STATION
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      isRedAlliance = true;
    } else {
      isRedAlliance = false;
    }
  }

  @Override
  public void execute() {
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
