// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FuelShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDShootAll extends ParallelCommandGroup {
  /** Creates a new PIDShootAll. */
  public PIDShootAll(FuelShooter fuelshooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinLeftMainPID(fuelshooter, Constants.Shooter.MAIN_MOTOR_RPM),
      new SpinMidMainPID(fuelshooter, Constants.Shooter.MAIN_MOTOR_RPM),
      new SpinRightMainPID(fuelshooter, Constants.Shooter.MAIN_MOTOR_RPM),
      new SpinLeftTopPID(fuelshooter, Constants.Shooter.TOP_MOTOR_RPM),
      new SpinRightTopPID(fuelshooter, Constants.Shooter.TOP_MOTOR_RPM)
    );
  }
}
