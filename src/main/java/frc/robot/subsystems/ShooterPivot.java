// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// Some of this code is 2024/2025 code, and may not be relevant
// to our robot. It is left in case we need it in the future.
// for example, the SparkMax code is not currently used, but it is left in case we need it in the future.

public class ShooterPivot extends SubsystemBase {

  private TalonFX shooterPivotMotor;
  private TalonFXConfiguration motorConfig;

  private PositionVoltage m_positionRequest;;

  private boolean isShooterPivotExtException = false;
  private boolean isShooterPivotRetException = false;
  private DigitalInput ShooterExtLimit;
  private DigitalInput ShooterRetLimit;

  private PositionVoltage m_request;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {

    shooterPivotMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_PIVOT);
    // --- Configuration ---
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // Change to Clockwise_Positive if needed
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    // Configure onboard PID for PositionVoltage
    var slot0Configs = motorConfig.Slot0;  // start with 0, 0, 0
    slot0Configs.kP = Constants.ShooterPvt.KP; // TODO tune
    slot0Configs.kI = Constants.ShooterPvt.KI;
    slot0Configs.kD = Constants.ShooterPvt.KD;

    shooterPivotMotor.getConfigurator().apply(motorConfig);

    m_positionRequest = new PositionVoltage(0).withSlot(0);

    try {
      // This tries to make a new digital input, and if it fails, throws an error
      ShooterExtLimit = new DigitalInput(Constants.ShooterPvt.DIO_EXT_LIMIT);
    } catch (Exception e) {
      isShooterPivotExtException = true;
      SmartDashboard.putBoolean(
          "exception thrown for Shooter Extend limit: ",
          isShooterPivotExtException);
    }

    try {
      // This sets a bottom limit for the shooter, and if it fails, throws an error
      ShooterRetLimit = new DigitalInput(Constants.ShooterPvt.DIO_RET_LIMIT);
    } catch (Exception e) {
      isShooterPivotRetException = true;
      SmartDashboard.putBoolean(
          "exception thrown for Shooter Retract limit: ",
          isShooterPivotRetException);
    }
  }

  // methods start here


  public double getEncoderRevs() {
    // getPosition() returns a StatusSignal; .getValueAsDouble() gets the rotation count
    //Position of the device in mechanism rotations. 
    //This can be the position of a remote sensor and is affected by the RotorToSensorRatio 
    //and SensorToMechanismRatio configs, as well as calls to setPosition.
    return shooterPivotMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder() {
    // Sets the current position to 0 rotations
    shooterPivotMotor.setPosition(0);
  }

  public void stopShooterPivot() {
    shooterPivotMotor.stopMotor();
  }

  public double getShooterPivotVelocity() {
    //returns shooter speed in rotations per second
    return shooterPivotMotor.getVelocity().getValueAsDouble();
  }

  
  public double getMotorSpeed()
    {    // returns speed between -1 and 1 
        return shooterPivotMotor.get();
    }

      // public boolean isShooterExtLimit() {
  //   if (isShooterPivotExtException) {
  //     return true;
  //   } else {
  //     return ShooterExtLimit.get();
  //   }
  // }

  public boolean isShooterExtLimit() {
    return isShooterPivotExtException ? true : ShooterExtLimit.get();
  }

  // public boolean isShooterRetLimit() {
  //   if (isShooterPivotRetException) {
  //     return true;
  //   } else {
  //     return ShooterRetLimit.get();
  //   }
  // }

  public boolean isShooterRetLimit() {
    return isShooterPivotRetException ? true : ShooterRetLimit.get();
  }

  // public boolean isFullyExtended() {
  //   return (getShooterEncoder() <= Constants.ShooterPivot.ENC_REVS_MAX);
  // }

  public boolean isFullyExtended() {
    return (getEncoderRevs() >= Constants.ShooterPvt.ENC_REVS_MAX);
  }

  public boolean isFullyRetracted()
    {   //want to zero the encoder when this limit is hit  
        return ShooterRetLimit.get();
    }

 
  private void manualSetSpeed (double speed)
    {   // sets speed between -1 and 1
        shooterPivotMotor.set(speed);
    }

  public void manualSetSpeedSafe(double speed)
    {
        if (isFullyRetracted())
        {
            resetEncoder();
            stopShooterPivot();
        } 
        else if (isFullyExtended()) 
        {
            stopShooterPivot();
        } 
        else 
        {
            manualSetSpeed(speed);
        }
    }

  
  public void pidSetPosition(double targetRevs) {
    //NOTE TO CODERS:  In a java method that return nothing ("void" returned),
    //                 a "return" line causes the code to exit the method
    //                 and not perform anything after that line
    // Clamp target to software limits
    //TODO ENSURE TARGET REVS >0 for logic to work!!!
    targetRevs =
        Math.max(0.0, Math.min(targetRevs, Constants.ShooterPvt.ENC_REVS_MAX));

    // Prevent driving further into limits
    if (targetRevs > getEncoderRevs() && isShooterExtLimit()) {
      stopShooterPivot();
      return; //causes the method to end here when the "if" is true
    }

    if (targetRevs < getEncoderRevs() && isShooterRetLimit()) {
      stopShooterPivot();
      resetEncoder();
      return;//causes the method to end here when the "if" is true
    }
    shooterPivotMotor.setControl(
        m_positionRequest.withPosition(targetRevs));
  }

  // Begin things that may not be relevant
  // these are things that might be useful in the future if we use CANSparkMax PID
  // we are not currently using it

  // !!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID
  // **** CHANGED BACK TO USING WPILib PID ****
  // **** due to spurious encoder polarity changes when run multiple autos in a row ****

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
        "Shooter Pivot Extend Limit?",
        isShooterExtLimit());
    SmartDashboard.putBoolean(
        "Shooter Pivot Retract Limit?",
        isShooterRetLimit());
    SmartDashboard.putNumber(
        "# Shooter Pivot Encoder Revolutions",
        getEncoderRevs());
    SmartDashboard.putBoolean(
        "Shooter Pivot is fully extended?",
        isFullyExtended());
  }
}
