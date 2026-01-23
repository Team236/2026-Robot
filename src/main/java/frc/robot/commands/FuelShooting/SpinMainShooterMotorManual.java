// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FuelShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinMainShooterMotorManual extends Command {

  private FuelShooter fuelShooter;
  private double mainMotorSetSpeed;

  /** Creates a new SpinShooterMotors. */
  public SpinMainShooterMotorManual(FuelShooter fuelShooter, double mainMotorSetSpeed ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.fuelShooter = fuelShooter; 
    this.mainMotorSetSpeed = mainMotorSetSpeed;

    addRequirements(fuelShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
  fuelShooter.spinMainMotor(mainMotorSetSpeed);


    double actualSpeed = fuelShooter.getAvgSpeed();
    
    SmartDashboard.putNumber("Speed asked: ", mainMotorSetSpeed);
    SmartDashboard.putNumber("Active speed", actualSpeed);
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