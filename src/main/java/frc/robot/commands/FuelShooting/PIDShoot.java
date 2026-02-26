// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.PreFeeder.PIDPrefeeder;
import frc.robot.commands.PreFeeder.RunPreFeeder;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.PreFeeder;
import frc.robot.subsystems.Swerve;

//Runs Main shooter motors right away, and waits a bit to run the Prefeeder motor in parallel,
//so the Main motors have a chance to come up to speed before fuel arrives

public class PIDShoot extends ParallelCommandGroup {
  /** Creates a new PIDMainandFeed*/
  public PIDShoot(MainRoller mainRoller, Swerve s_Swerve, PreFeeder preFeeder) {
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> mainRoller.useInitialBoost = true),
        new WaitCommand(0.4),
        new InstantCommand(() -> mainRoller.useInitialBoost = false)
      ),

      new PIDMainRoller(mainRoller, s_Swerve, Constants.ShooterConstants.MAIN_MOTOR_RPM),

      new SequentialCommandGroup(
        new WaitCommand(0.3),
        new PIDPrefeeder(preFeeder, Constants.PreFeederConstants.DESIRED_RPM)
      )
    );
}
}

