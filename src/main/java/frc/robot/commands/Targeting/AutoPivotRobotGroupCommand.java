package frc.robot.commands.Targeting;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPivotRobotGroupCommand extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;
  
  private DoubleSupplier rotationSupplier; 

  public AutoPivotRobotGroupCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup, DoubleSupplier rotationSupplier) {
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;
    this.rotationSupplier = rotationSupplier;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
      
      double newRotation = rotationSupplier.getAsDouble();

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        MathUtil.clamp(newRotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity),
        !robotCentricSup.getAsBoolean(),
        true
    );
  }

  @Override
  public boolean isFinished() { return false; }
}