// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Floor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Floor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDFloor extends Command {
 
  private Floor floor;
  private double floorRPM, floorRPS;

//MUST PASS IN DESIRED VELOCITY IN RPM IN THIS COMMAND
  public PIDFloor(Floor floor, double floorROM) {
    this.floor = floor;
    this.floorRPM = floorRPM;
    addRequirements(this.floor);
  }

  @Override
  public void initialize() {
  floorRPS = floorRPM / 60.0;
  }

  @Override
  public void execute() {
    floor.floorPID(floorRPS);

  }

  @Override
  public void end(boolean interrupted) {
    floor.stopFloor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
