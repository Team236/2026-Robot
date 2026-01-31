// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.TopRoller;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDShoot extends ParallelCommandGroup {
  /** Creates a new PIDShoot. */
  public PIDShoot(MainRoller mainRoller, TopRoller topRoller) {
    addCommands(
     new PIDMainRoller(mainRoller, Constants.Shooter.MAIN_MOTOR_RPM),
     new PIDTopRoller(topRoller, Constants.Shooter.TOP_MOTOR_RPM)
    );
  }
}

