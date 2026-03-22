package frc.robot.subsystems;

import java.util.Queue;
import java.util.Set;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.BinRelease.PIDMove;

public class BinRelease extends SubsystemBase {
    
    private TalonFX binReleaseMotor;
    private TalonFXConfiguration motorConfig;

    private DigitalInput maxRetractLimit; //limit switch
    private DigitalInput maxExtendLimit;  //limit switch
    private boolean isBinRetException;
    private boolean isBinExtException;

    private PositionVoltage m_extendRequest;
    private PositionVoltage m_retractRequest;
    private MotionMagicVoltage m_magicRequest;

    private double agitateSetpoint;

    /** Creates a new BinRelease. */
    //This system uses a motor to extend and retract the bin that holds the fuel
    public BinRelease() {
        binReleaseMotor = new TalonFX(Constants.MotorControllers.ID_BIN_REL, "rio"); //will be rio not usb

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //set required slot0 gains, for PID Position control with TalonFX
        var slot0Configs = motorConfig.Slot0;  // start with 0, 0, 0
        slot0Configs.kP = Constants.BinReleaseConstants.KP_BIN_EXTEND;  
        slot0Configs.kI = Constants.BinReleaseConstants.KI_BIN_EXTEND; 
        slot0Configs.kD = Constants.BinReleaseConstants.KD_BIN_EXTEND; 

        var slot1Configs = motorConfig.Slot1;
        slot1Configs.kP = Constants.BinReleaseConstants.KP_BIN_RETRACT;
        slot1Configs.kI = Constants.BinReleaseConstants.KI_BIN_RETRACT; 
        slot1Configs.kD = Constants.BinReleaseConstants.KD_BIN_RETRACT;


        Slot2Configs slot2Configs = motorConfig.Slot2;
        // TUNE THESE -- COPIED FROM 2025 FOR NOW
        slot2Configs.kS = 0;//0.25; // Add 0.25 V output to overcome static friction
        slot2Configs.kV = 0;//0.12; // A velocity target of 1 rps results in 0.12 V output
        slot2Configs.kA = 0;//0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot2Configs.kP = 0;//2;//4.8; // A position error of 2.5 rotations results in 12 V output
        slot2Configs.kI = 0;//0; // no output for integrated error
        slot2Configs.kD = 0;//0.1; // A velocity error of 1 rps results in 0.1 V output

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
    // TUNE THESE -- COPIED FROM 2025 FOR NOW
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;//80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 120;//160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1200;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        binReleaseMotor.getConfigurator().apply(motorConfig);
        m_extendRequest = new PositionVoltage(0).withSlot(0);
        m_retractRequest = new PositionVoltage(0).withSlot(1);
        m_magicRequest = new MotionMagicVoltage(0).withSlot(2);

        // attempts to make retr. limit switch, if it fails, throws an error
        try{
           maxRetractLimit = new DigitalInput(Constants.BinReleaseConstants.DIO_RET_LIMIT);
        } 
        catch (Exception e){
           isBinRetException = true;
           SmartDashboard.putBoolean("Exception thrown for binReleaseMaxRetractLimit: ", isBinRetException);
        }
        
        // attempts to make ext. limit switch, if it fails, throws an error
        try{
           maxExtendLimit = new DigitalInput(Constants.BinReleaseConstants.DIO_EXT_LIMIT);
        } 
        catch (Exception e){
           isBinRetException = true;
           SmartDashboard.putBoolean("Exception thrown for binReleaseMaxExtendLimit: ", isBinExtException);
        }

    }

    //METHODS START HERE:

    public void resetEncoder(){
        binReleaseMotor.setPosition(0.0);
        }

    public double getEncoderRevolutions(){
        return binReleaseMotor.getPosition().getValueAsDouble();
        }

    public void stopMotor(){  
        binReleaseMotor.set(0); //set the speed to 0
       // binReleaseMotor.stopMotor();//this is another way to stop it
        }

    // returnsspeed between -1 and 1
    public double getMotorSpeed(){   
        return binReleaseMotor.get();
        }

    // sets speed between -1 and 1
    public void manualSetSpeed(double speed){
        binReleaseMotor.set(speed);
        }

    public boolean isFullyRetracted(){   
        //want to zero the encoder when this limit is hit
        return maxRetractLimit.get(); // have to add not (!) because on the robot it is by default true
    }

    public boolean isFullyExtended(){   
        // TBD make sure encoder reading is increasing as mechanism extends, so the ">" sign works below
        // return (maxExtendLimit.get() || getEncoderRevolutions() > Constants.BinReleaseConstants.ENC_REVS_MAX);
        return (getEncoderRevolutions() > Constants.BinReleaseConstants.ENC_REVS_MAX);
    }

    public void manualSetSpeedSafe(double speed){
        if (isFullyRetracted() && speed < 0) {
            SmartDashboard.putBoolean("Full Retract so reset encoder", isFullyRetracted());
            resetEncoder();
            stopMotor();
          } 
        else if (isFullyExtended() && speed >= 0) {
            stopMotor();
          } 
        else {
            manualSetSpeed(speed);
          }
        }

  
    //uses position control with Kp, Ki and Kd to bring the motor to the desired encoder revolutions  
    public void PIDControlToPosition(double desiredRevs) {
        if (isFullyRetracted() && (desiredRevs < getEncoderRevolutions())) {
            resetEncoder();
            stopMotor();
        }
        else if ((isFullyExtended() &&  desiredRevs > getEncoderRevolutions()) ) {
        // else if (isFullyExtended() && (desiredRevs > getEncoderRevolutions())) {
            stopMotor();
        } 
        else {
            if (desiredRevs > getEncoderRevolutions()) {
                binReleaseMotor.setControl(m_extendRequest.withPosition(desiredRevs)); 
            } else {
                binReleaseMotor.setControl(m_retractRequest.withPosition(desiredRevs)); 
            }
        }
    }

    public void doMotionMagic(double desiredRevs) {
        if (binReleaseMotor.get() > 0 && isFullyExtended()) {
            stopMotor();
            return;
        } else if (binReleaseMotor.get() < 0 && isFullyRetracted()) {
            resetEncoder();
            stopMotor();
        } else {
            binReleaseMotor.setControl(m_magicRequest.withPosition(desiredRevs));
        }
    }

    public Command getAgitateCommand() {
        return 
            new PIDMove(this, Constants.BinReleaseConstants.BIN_DOWN_POSSITION - 8)
            .until(() -> Math.abs(this.getEncoderRevolutions() - (Constants.BinReleaseConstants.BIN_DOWN_POSSITION - 8)) < 0.2)
            .andThen(
                new PIDMove(this, Constants.BinReleaseConstants.BIN_DOWN_POSSITION)
                .until(() -> Math.abs(this.getEncoderRevolutions() - Constants.BinReleaseConstants.BIN_DOWN_POSSITION) < 0.2)
            )
            .repeatedly()
            .finallyDo(() -> this.PIDControlToPosition(Constants.BinReleaseConstants.BIN_DOWN_POSSITION));
    }

    // public Command getRisingAgitateCommand() {
    //     return
    //         new InstantCommand(() -> this.agitateSetpoint = this.getEncoderRevolutions() - 5)
    //         .andThen(new PIDMove(this, this.agitateSetpoint)
    //         .until(() -> Math.abs(this.getEncoderRevolutions() - this.agitateSetpoint) < 0.2))
    //         .andThen(new InstantCommand(() -> this.agitateSetpoint = this.getEncoderRevolutions() + 2.5))
    //         .andThen(new PIDMove(this, this.agitateSetpoint))
    //         .until(() -> Math.abs(this.getEncoderRevolutions() - this.agitateSetpoint) < 0.2)
    //         .repeatedly();
    // }

        public Command getRisingAgitateCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> this.agitateSetpoint = this.getEncoderRevolutions() - 5),
            
            new DeferredCommand(() -> 
                new PIDMove(this, this.agitateSetpoint)
                    .until(() -> Math.abs(this.getEncoderRevolutions() - this.agitateSetpoint) < 0.2), 
                Set.of(this)
            ),
            
            new InstantCommand(() -> this.agitateSetpoint = this.getEncoderRevolutions() + 2.5),
            
            new DeferredCommand(() -> 
                new PIDMove(this, this.agitateSetpoint)
                    .until(() -> Math.abs(this.getEncoderRevolutions() - this.agitateSetpoint) < 0.2), 
                Set.of(this)
            )
        ).repeatedly();
    }

    public Command getManualRisingAgitateCommand() {
        return new DeferredCommand(() -> {
            var state = new Object() { double baseline = getEncoderRevolutions(); };
            double finalTarget = 0.0; 

            return new SequentialCommandGroup(
                new RunCommand(() -> this.manualSetSpeedSafe(-0.6), this)
                    .until(() -> this.getEncoderRevolutions() <= state.baseline - 7.5),
                
                new InstantCommand(() -> state.baseline -= 5.0), 
                new RunCommand(() -> this.manualSetSpeedSafe(0.35), this) 
                    .until(() -> this.getEncoderRevolutions() >= state.baseline)
            )
            .repeatedly()
            .until(() -> this.getEncoderRevolutions() <= finalTarget + 5.0)
            .andThen(this.getManualTopAgitateCommand())
            .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

        public Command getManualAgitateCommand() {
        return new DeferredCommand(() -> {
            double binOutPose = Constants.BinReleaseConstants.BIN_DOWN_POSSITION;
            double finalTarget = Constants.BinReleaseConstants.BIN_DOWN_POSSITION - 7.5;

            return new SequentialCommandGroup(
                new RunCommand(() -> this.manualSetSpeedSafe(-0.6), this)
                    .until(() -> this.getEncoderRevolutions() <=finalTarget),
                
                new RunCommand(() -> this.manualSetSpeedSafe(0.35), this) 
                    .until(() -> this.getEncoderRevolutions() >= binOutPose)
            )
            .repeatedly()
            .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

    public Command getManualTopAgitateCommand() {
        return new DeferredCommand(() -> {
            double binTopPose = Constants.BinReleaseConstants.BIN_AGITATE_UP_POSSITION;
            
            double finalTarget = binTopPose + 7.5;

            return new SequentialCommandGroup(
                new RunCommand(() -> this.manualSetSpeedSafe(0.35), this)
                    .until(() -> this.getEncoderRevolutions() >= finalTarget),
                
                new RunCommand(() -> this.manualSetSpeedSafe(-0.6), this) 
                    .until(() -> this.getEncoderRevolutions() <= binTopPose)
            )
            .repeatedly()
            .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Bin encoder revolutions:", getEncoderRevolutions());
        SmartDashboard.putBoolean("Bin fully extended:", isFullyExtended());
        SmartDashboard.putBoolean("Bin fully retracted:", isFullyRetracted());
    }
}
