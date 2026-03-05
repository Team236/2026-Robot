// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FuelShooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Floor.RunFloor;
import frc.robot.commands.PreFeeder.PIDPrefeeder;
import frc.robot.commands.PreFeeder.RunPreFeeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.PreFeeder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousStartup extends ParallelCommandGroup {
  /** Creates a new AutonomousStartup. */
  public AutonomousStartup(PreFeeder preFeeder, MainRoller mainRoller, Floor floor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ManualMainRoller(mainRoller, -0.35),
      new RunPreFeeder(preFeeder, -0.35),
      new RunFloor(floor, 0.35)
      // eventuall add the PID BIN OUT here?
    );
  }
}
