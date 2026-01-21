package frc.robot.subsystems;



import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BinRelease extends SubsystemBase {
    
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    private DigitalInput maxRetractLimit; //limit switch
    private DigitalInput maxExtendLimit;  //limit switch
    private boolean isBinRetException;
    private boolean isBinExtException;

    private PIDController pidController;

    private double desiredSpeed;
    private double desiredRevolutions;

    public BinRelease() {

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd

        motor = new TalonFX(0); //tbd
        motor.getConfigurator().apply(motorConfig);

        pidController = new PIDController(0, 0, 0); //tbd

    try {
      //  This tries to make a new digital input, and if it fails, throws an error 
       maxRetractLimit = new DigitalInput(Constants.BinRelease.DIO_RET_LIMIT); //tbd
    } catch (Exception e) {
       isBinRetException = true;
      SmartDashboard.putBoolean("exception thrown for Bin Retract limit: ", isBinRetException);
    }
 
   try {
      //  This tries to make a new digital input, and if it fails, throws an error 
       maxExtendLimit = new DigitalInput(Constants.BinRelease.DIO_EXT_LIMIT); //tbd
    } catch (Exception e) {
       isBinRetException = true;
      SmartDashboard.putBoolean("exception thrown for Bin Extend limit: ", isBinRetException);
    }

    }


//METHODS START HERE:

    public void resetEncoder()
    {
        motor.setPosition(0.0);
    }

    public double getEncoderRevolutions()
    {
        return motor.getPosition().getValueAsDouble();
    }

    public void stopMotor()
    {  // desired speed is zero
        motor.set(0);
    }

    public double getMotorSpeed()
    {   //returns speed between -1 and +1
        return motor.get();
    }

    private void setMotorSpeed(double speed)
    {  //sets the motor speed to "speed" which is passed in, between -1 and +1
        motor.set(speed);
    }

    public boolean isFullyRetracted()
    {   //want to zero the encoder when this limit is hit
        return maxRetractLimit.get();
    }

    public boolean isFullyExtended()
    {   //TBD  make sure encoder reading is increasing as mechanism extends, so the ">" sign works below
        return (maxExtendLimit.get() || getEncoderRevolutions() > Constants.BinRelease.ENC_REVS_MAX); //set to a high value at first, for code testing
    }

    public void setMotorSpeedSafe(double speed)
    {
        if (isFullyRetracted())
        {
            resetEncoder();
            stopMotor();
        } 
        else if (isFullyExtended()) 
        {
            stopMotor();
        } 
        else 
        {
            setMotorSpeed(speed);
        }
    }

    public void setDesiredPosition(double revolutions)
    {
        desiredRevolutions = revolutions;
    }

    public void doPIDControl()
    {
        //double revolutionsError = desiredRevolutions - getEncoderRevolutions();

        //double speed = pidController.calculate()
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Bin fully extended:", isFullyExtended());
        SmartDashboard.putBoolean("Bin fully retracted:", isFullyRetracted());
    }
}
