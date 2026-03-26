// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDMainRoller extends Command {

  private MainRoller mainRoller;
  private Swerve s_Swerve;
  private double mainMotorSetRPM, mainRPS;
  private final Timer shootTimer = new Timer();

  /** Creates a new PIDMainRoller */

  //MUST PASS IN DESIRED VELOCITY IN RPM IN THIS COMMAND
  public PIDMainRoller(MainRoller mainRoller, Swerve s_Swerve, double mainMotorSetRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mainRoller = mainRoller; 
    this.s_Swerve = s_Swerve;
    this.mainMotorSetRPM = mainMotorSetRPM; 
    addRequirements(mainRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   mainRPS = mainMotorSetRPM / 60.0;
   shootTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
  // mainRoller.MainPID(mainRPS);

  // mainRoller.MainPID(mainRoller.calculateRPM(s_Swerve.getDistanceToHub()) / 60.0);

  if (s_Swerve.inNeutralZone()) {
    mainRoller.MainPID(mainRoller.calculateNuetralRPM(s_Swerve.getXtoHub()) / 60.0);
  } else{
    if (mainRoller.useInitialBoost) {
      mainRoller.MainPID((mainRoller.calculateRPM(s_Swerve.getDistanceToHub()) + 80) / 60.0);
    } else {
      mainRoller.MainPID(mainRoller.calculateRPM(s_Swerve.getDistanceToHub()) / 60.0);
    }
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootTimer.stop();
    SmartDashboard.putNumber("Bin Tuning/ Shot Time", shootTimer.get());

    mainRoller.stopMain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}