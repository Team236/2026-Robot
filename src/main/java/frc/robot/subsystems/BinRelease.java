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

public class BinRelease extends SubsystemBase {
    
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    private DigitalInput maxRetractionDI;
    private DigitalInput maxExtensionDI;

    private PIDController pidController;

    private double desiredSpeed;
    private double desiredRevolutions;

    public BinRelease() {

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 0; //tbd

        motor = new TalonFX(0); //tbd
        motor.getConfigurator().apply(motorConfig);

        pidController = new PIDController(0, 0, 0); //tbd

        maxRetractionDI = new DigitalInput(0); //tbd
        maxExtensionDI = new DigitalInput(0); //tbd
    }

    public void resetEncoder()
    {
        motor.setPosition(0.0);
    }

    public double getEncoderRevolutions()
    {
        return motor.getPosition().getValueAsDouble();
    }

    public void stopMotor()
    {
        desiredSpeed = 0;
        motor.set(0);
    }

    public double getMotorSpeed()
    {
        return motor.get();
    }

    private void setMotorSpeed(double speed)
    {
        desiredSpeed = speed;
        motor.set(speed);
    }

    public boolean isFullyRetracted()
    {
        return maxRetractionDI.get() || getEncoderRevolutions() < 0;
    }

    public boolean isFullyExtended()
    {
        return maxExtensionDI.get() || getEncoderRevolutions() > 0;
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
