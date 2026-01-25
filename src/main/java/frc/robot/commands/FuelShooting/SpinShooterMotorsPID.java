// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinShooterMotorsPID extends Command {

  private FuelShooter fuelShooter;
  private double mainMotorSetRPM;
  private double topMotorSetRPM;

  /** Creates a new SpinShooterMotors. */
  public SpinShooterMotorsPID(FuelShooter fuelShooter, double mainMotorSetRPM, double topMotorSetRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.fuelShooter = fuelShooter; 
    this.mainMotorSetRPM = mainMotorSetRPM; //tbd
    this.topMotorSetRPM = topMotorSetRPM; //tbd

    addRequirements(fuelShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
  double mainRPS = mainMotorSetRPM / 60.0;
  double topRPS = topMotorSetRPM / 60.0;
  fuelShooter.shootFuel(mainRPS, topRPS); 
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fuelShooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}