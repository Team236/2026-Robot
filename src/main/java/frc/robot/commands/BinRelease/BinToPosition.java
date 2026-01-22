package frc.robot.commands.BinRelease;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BinRelease;

public class BinToPosition extends Command {
    
    private BinRelease binRelease;
    private double desiredRevs;

    public BinToPosition(BinRelease binRelease, double desiredRevs)
    {
        this.binRelease = binRelease;
        this.desiredRevs = desiredRevs;
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
        return false;
    }
}
