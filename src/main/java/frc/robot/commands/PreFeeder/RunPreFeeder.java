// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PreFeeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PreFeeder;

public class RunPreFeeder extends Command {
  private PreFeeder preFeeder;
  private double speed;

  public RunPreFeeder(PreFeeder feeder, double speed) {
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
    SmartDashboard.putNumber("attempting prefeeder speed", speed);
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
