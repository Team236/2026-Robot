package frc.robot.commands.BinRelease;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BinRelease;

public class PIDMove extends Command {
    
    private BinRelease binRelease;
    private double desiredRevs;

    public PIDMove(BinRelease binRelease, double desiredRevs)
    {
        this.binRelease = binRelease;
        this.desiredRevs = desiredRevs;
        addRequirements(this.binRelease);
    }

    @Override 
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        binRelease.PIDControlToPosition(desiredRevs);
    }

    @Override
    public void end(boolean interrupted)
    {
        binRelease.stopMotor();
    }

    @Override
    public boolean isFinished()
    {
        return binRelease.isFullyExtended();
    }
}
