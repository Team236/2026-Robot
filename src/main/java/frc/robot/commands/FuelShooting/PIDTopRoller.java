// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TopRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDTopRoller extends Command {
  private TopRoller topRoller;
  private double topMotorSetRPM, topRPS;

  /** Creates a new PIDTopRoller */
  public PIDTopRoller(TopRoller topRoller, double topMotorSetRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.topRoller = topRoller; 
    this.topMotorSetRPM = topMotorSetRPM; //tbd

    addRequirements(topRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   topRPS = topMotorSetRPM / 60.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
  topRoller.TopPID(topRPS);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    topRoller.stopTop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}