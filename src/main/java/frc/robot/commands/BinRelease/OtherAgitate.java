// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BinRelease;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BinRelease;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OtherAgitate extends Command {
  private BinRelease binRelease;
  private double position;
  /** Creates a new OtherAgitate. */
  public OtherAgitate(BinRelease binRelease, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.binRelease = binRelease;
    this.position = position;
    addRequirements(binRelease);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    binRelease.manualSetSpeedSafe(Constants.BinReleaseConstants.AGITATE_TEST_SPEED * Math.signum(position - binRelease.getEncoderRevolutions()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    binRelease.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(binRelease.getEncoderRevolutions() - position) < Constants.BinReleaseConstants.AGITATE_TOLERANCE; // distance within tolerance
  }
}
