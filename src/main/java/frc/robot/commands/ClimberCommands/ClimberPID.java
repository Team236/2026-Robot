// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberPID extends Command {
  private Climber climber;
  private double desiredRevs;

  public ClimberPID(Climber climber, double desiredRevs)
  {
    this.climber = climber;
    this.desiredRevs = desiredRevs;
    addRequirements(this.climber);
  }

  @Override 
  public void initialize() {
    if (climber.isBottomLimit()) {
      climber.resetClimberEncoder();
    }
  }

  @Override
  public void execute()
  {
    climber.PIDControlToPosition(desiredRevs);
  }

  @Override
  public void end(boolean interrupted)
  {
    climber.stopClimber();
  }

  @Override
  public boolean isFinished()
  {
    return climber.isTopLimit();
  }
}
