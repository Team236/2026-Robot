// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDIntake extends Command {
 
  private Intake  intake;
  private double intakeRPS, intakeRPM;

//MUST PASS IN DESIRED VELOCITY IN RPM IN THIS COMMAND
  public PIDIntake(Intake intake, double intakeRPM) {
    this.intake = intake;
    this.intakeRPM = intakeRPM;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
  intakeRPS = intakeRPM / 60.0;
  }

  @Override
  public void execute() {
    intake.IntakePID(intakeRPS);

  }

  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
