package frc.robot.commands.Targeting;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;

public class AutoPivotShooterGroupCommand extends Command {
  private MainRoller mainRoller;
  private ShooterPivot shooterPivot;
  

  private DoubleSupplier distanceSupplier; 

  public AutoPivotShooterGroupCommand(Swerve s_Swerve, MainRoller mainRoller, ShooterPivot shooterPivot, DoubleSupplier distanceSupplier) {
    this.mainRoller = mainRoller;
    this.shooterPivot = shooterPivot;
    this.distanceSupplier = distanceSupplier; 

    addRequirements(mainRoller, shooterPivot);
  }

  @Override
  public void execute() {
    double newDistance = distanceSupplier.getAsDouble(); 

    double targetRPM = mainRoller.calculateRPM(newDistance);
    double targetRevs = shooterPivot.calculateHoodAngle(newDistance);

    mainRoller.MainPID(targetRPM);
    shooterPivot.pidSetPosition(targetRevs);
  }

  @Override
  public void end(boolean interrupted) {
    mainRoller.stopMain();
    shooterPivot.stopShooterPivot();
  }

  @Override
  public boolean isFinished() { return false; }
}