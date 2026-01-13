// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PathPlanner;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialPathTest2 extends SequentialCommandGroup {
  /** Creates a new SequentialPathTest. */
  public SequentialPathTest2(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.defer(() -> s_Swerve.followPathCommand("SequentialPart3"), Set.of(s_Swerve)),
      new WaitCommand(2),
      Commands.defer(() -> s_Swerve.followPathCommand("SequentialPart4"), Set.of(s_Swerve)) 
    );
  }
}
