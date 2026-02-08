package frc.robot.commands.Targeting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;

public class TargetRobotForShoot extends ParallelCommandGroup {
  
  public TargetRobotForShoot(Swerve s_Swerve, MainRoller mainRoller, ShooterPivot shooterPivot, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
    
    addCommands(
        new AutoPivotTowardHub(s_Swerve, translationSup, strafeSup, robotCentricSup),
        
        new AutoPivotShooter(s_Swerve, mainRoller, shooterPivot)
    );
  }
}