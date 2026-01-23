package frc.robot.commands.BinRelease;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BinRelease;

public class ManualMove extends Command 
{
    private BinRelease binRelease;
    private double speed;

    public ManualMove(BinRelease binRelease, double speed)
    {
        this.binRelease = binRelease;
        this.speed = speed;
        addRequirements(this.binRelease);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        binRelease.manualSetSpeedSafe(speed);
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
