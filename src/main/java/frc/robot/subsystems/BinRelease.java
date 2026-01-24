package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BinRelease extends SubsystemBase {
    
    private TalonFX binReleaseMotor;
    private TalonFXConfiguration motorConfig;

    private DigitalInput maxRetractLimit; //limit switch
    private DigitalInput maxExtendLimit;  //limit switch
    private boolean isBinRetException;
    private boolean isBinExtException;

   // private PIDController pidController;
    private PositionVoltage m_request;


    public BinRelease() 
    {
        binReleaseMotor = new TalonFX(Constants.MotorControllers.ID_BIN_REL, "usb"); //tbd - will all be USB???

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //set required slot0 gains, for PID Position control with TalonFX
        var slot0Configs = motorConfig.Slot0;  // start with 0, 0, 0
        slot0Configs.kP = Constants.BinRelease.KP; // 2.4; 
        slot0Configs.kI = Constants.BinRelease.KI; // 0
        slot0Configs.kD = Constants.BinRelease.KD; // 0.1 

        
        binReleaseMotor.getConfigurator().apply(motorConfig);

        m_request = new PositionVoltage(0).withSlot(0);
        // pidController = new PIDController(0, 0, 0); //tbd

        // attempts to make ret limit switch, if it fails, throws an error
        try
        {
           maxRetractLimit = new DigitalInput(Constants.BinRelease.DIO_RET_LIMIT);
        } 
        catch (Exception e)
        {
           isBinRetException = true;
           SmartDashboard.putBoolean("Exception thrown for binReleaseMaxRetractLimit: ", isBinRetException);
        }

        // attempts to make ext limit switch, if it fails, throws an error
        try
        {
           maxExtendLimit = new DigitalInput(Constants.BinRelease.DIO_EXT_LIMIT);
        } 
        catch (Exception e)
        {
           isBinRetException = true;
           SmartDashboard.putBoolean("Exception thrown for binReleaseMaxExtendLimit: ", isBinExtException);
        }
    }

    //METHODS START HERE:

    public void resetEncoder()
    {
        binReleaseMotor.setPosition(0.0);
    }

    public double getEncoderRevolutions()
    {
        return binReleaseMotor.getPosition().getValueAsDouble();
    }

    public void stopMotor()
    {  
        // desired speed is zero
        binReleaseMotor.set(0);
    }

    // returns speed between -1 and 1
    public double getMotorSpeed()
    {   
        return binReleaseMotor.get();
    }

    // sets speed between -1 and 1
    private void manualSetSpeed(double speed)
    {
        binReleaseMotor.set(speed);
    }

    public boolean isFullyRetracted()
    {   
        //want to zero the encoder when this limit is hit
        return maxRetractLimit.get();
    }

    public boolean isFullyExtended()
    {   
        // TBD make sure encoder reading is increasing as mechanism extends, so the ">" sign works below
        return (maxExtendLimit.get() || getEncoderRevolutions() > Constants.BinRelease.ENC_REVS_MAX); //set to a high value at first, for code testing
    }


    //EDC - renamed method below 1/23/26:
    public void manualSetSpeedSafe(double speed)
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
            manualSetSpeed(speed);
        }
    }

  
    public void PIDControlToPosition(double desiredRevs) {

        if (isFullyRetracted() && (desiredRevs < getEncoderRevolutions())) 
        {
            resetEncoder();
            stopMotor();
        } else if (isFullyExtended() && (desiredRevs > getEncoderRevolutions())) 
        {
            stopMotor();
        } else 
        {
            binReleaseMotor.setControl(m_request.withPosition(desiredRevs));   
        }

        //uses position control with Kp, Ki and Kd to bring the motor to the desired encoder revolutions  
        //double revolutionsError = desiredRevolutions - getEncoderRevolutions();
        //double speed = pidController.calculate()
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Bin encoder revolutions:", getEncoderRevolutions());
        SmartDashboard.putBoolean("Bin fully extended:", isFullyExtended());
        SmartDashboard.putBoolean("Bin fully retracted:", isFullyRetracted());
    }
}
