// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PreFeeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreFeeder;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDPrefeeder extends Command {
 
  private PreFeeder  preFeeder;
  private Swerve s_Swerve;
  private double preFeedRPS, preFeedRPM;

//MUST PASS IN DESIRED VELOCITY IN RPM IN THIS COMMAND
  public PIDPrefeeder(PreFeeder feeder, Swerve s_Swerve,double preFeedRPM) {
    this.preFeeder = feeder;
    this.s_Swerve = s_Swerve;
    this.preFeedRPM = preFeedRPM;
    addRequirements(this.preFeeder);
  }

  @Override
  public void initialize() {
  preFeedRPS = preFeedRPM / 60.0;
  }

  @Override
  public void execute() {
    preFeeder.PreFeederPID(preFeedRPS);
    preFeeder.PreFeederPID(preFeeder.calculateRPM(s_Swerve.getDistanceToHub()) / 60.0);
  }

  @Override
  public void end(boolean interrupted) {
    preFeeder.stopPreFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
