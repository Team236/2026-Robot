package frc.robot.commands.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.PreFeeder.PIDPrefeeder;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.PreFeeder;
import frc.robot.subsystems.ShooterPivot; // Replace with your actual Shooter subsystem
import frc.robot.subsystems.Swerve;

public class AimOnMove extends ParallelCommandGroup {

  public AimOnMove(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup, MainRoller mainRoller, ShooterPivot shooterPivot, PreFeeder preFeeder) {
    addCommands(
      new AutoPivotRobotGroupCommand(s_Swerve, translationSup, strafeSup, robotCentricSup, () -> s_Swerve.getRotationMoving()[1]),

      new SequentialCommandGroup(
          new WaitCommand(0.3),
        new ParallelCommandGroup(
          new AutoPivotShooterGroupCommand(s_Swerve, mainRoller, shooterPivot, () -> s_Swerve.getRotationMoving()[0]),
          new PIDPrefeeder(preFeeder, s_Swerve, Constants.PreFeederConstants.DESIRED_RPM)
        )
      )
    );
  }
}