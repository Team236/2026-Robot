// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.PreFeeder.RunPreFeeder;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.PreFeeder;
import frc.robot.subsystems.TopRoller;


public class ManualShoot extends ParallelCommandGroup {
  /** Creates a new ManualShoot. */
  public ManualShoot(MainRoller mainRoller, TopRoller topRoller, PreFeeder preFeeder) {
    
    addCommands(

     new ManualMainRoller(mainRoller, Constants.ShooterConstants.MAIN_MOTOR_SPEED),
    //  new ManualTopRoller(topRoller, Constants.Shooter.TOP_MOTOR_SPEED),

    new SequentialCommandGroup(
      new WaitCommand(0.25),//adjust - time to bring main shooter motors to speed
      new RunPreFeeder(preFeeder, Constants.PreFeederConstants.TEST_SPEED)
      )
    ); 
  }
}
