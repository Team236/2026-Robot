// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDPivot extends Command {
  
  private ShooterPivot shooterPivot;
  private double desiredRevs;

  public PIDPivot(ShooterPivot shooterPivot, double desiredRevs) {
    this.shooterPivot = shooterPivot;
    this.desiredRevs = desiredRevs;
    addRequirements(this.shooterPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterPivot.pidSetPosition(desiredRevs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPivot.stopShooterPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooterPivot.isFullyExtended() || shooterPivot.isShooterExtLimit());
  }
}