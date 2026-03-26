package frc.robot.commands.Targeting;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.commands.BinRelease.PIDMove;
import frc.robot.commands.ClimberCommands.ClimberPID;
import frc.robot.subsystems.BinRelease;
import frc.robot.subsystems.Climber;

public class AlignAndFullClimb extends SequentialCommandGroup {

  public AlignAndFullClimb(Swerve s_Swerve, Command climbCommand, Climber climber, BinRelease binRelease) {
    
    addCommands(

      new ParallelCommandGroup(
        Commands.defer(
          () -> s_Swerve.getClimbTargetingPath(), 
          Set.of(s_Swerve) 
        ),

        new SequentialCommandGroup(
          new PIDMove(binRelease, 0).until(() -> binRelease.getEncoderRevolutions() < 5),
          new ClimberPID(climber, Constants.ClimberConstants.PREP_CLIMBER_REVS).until(() -> climber.getClimberEncoder() > Constants.ClimberConstants.PREP_CLIMBER_REVS - 1)
        )

      ),
        
        new SequentialCommandGroup(
            Commands.defer(
                () -> s_Swerve.getFinishClimbCommand(), 
                Set.of(s_Swerve) 
            ),
  
            new ClimberPID(climber, Constants.ClimberConstants.CLIMB_L1_FRONT)
        )
    );
  }
}