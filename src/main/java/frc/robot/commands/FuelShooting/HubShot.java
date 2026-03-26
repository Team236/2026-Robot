package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.ShooterPivot;

public class HubShot extends ParallelCommandGroup {
  /** Creates a new HubShot. */
  public HubShot(ShooterPivot shooterPivot, MainRoller mainRoller) {
    addCommands(
      shooterPivot.run(() -> shooterPivot.pidSetPosition(5.0)),
      mainRoller.run(() -> mainRoller.spinMainMotor(2380.0))
    );
  }
}