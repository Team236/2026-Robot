package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BinRelease.PIDMove;
import frc.robot.commands.Targeting.AutoPivotShooterGroupCommand;
import frc.robot.subsystems.BinRelease;
import frc.robot.subsystems.Intake;

public class IntakeOvercurrentWithBinExtend extends ParallelCommandGroup {
  /** Creates a new IntakeWithBinExtend. */
  public IntakeOvercurrentWithBinExtend(BinRelease binRelease, Double desiredRevs, Intake intake, int intakeRPM) {

    addCommands(
      new PIDMove(binRelease, desiredRevs),
      new PIDIntakeOvercurrent(intake, intakeRPM)
    );
  }
}