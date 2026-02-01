// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PreFeeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreFeeder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDPrefeeder extends Command {
 
  private PreFeeder  preFeeder;
  private double speed;

  public PIDPrefeeder(PreFeeder feeder, double speed) {
    this.preFeeder = feeder;
    this.speed = speed;
    addRequirements(this.preFeeder);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    preFeeder.setPreFeederSpeed(speed);

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
