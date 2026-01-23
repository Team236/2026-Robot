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

public class shooterPivot extends SubsystemBase {

  private final TalonFX shooterPivotMotor;
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);

  private boolean isShooterPivotExtException = false;
  private boolean isShooterPivotRetException = false;

  private DigitalInput ShooterExtLimit;
  private DigitalInput ShooterRetLimit;

  // --- Configuration ---
  TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  /** Creates a new ShooterPivot. */
  public shooterPivot() {

    shooterPivotMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_PIVOT);

    motorConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // Change to Clockwise_Positive if needed
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterPivotMotor.getConfigurator().apply(motorConfig);

    // Configure onboard PID for PositionVoltage
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 20.0; // TODO tune
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kV = 0.0;

    shooterPivotMotor.getConfigurator().apply(slot0);

    try {
      // This tries to make a new digital input, and if it fails, throws an error
      ShooterExtLimit = new DigitalInput(Constants.ShooterPivot.DIO_EXT_LIMIT);
    } catch (Exception e) {
      isShooterPivotExtException = true;
      SmartDashboard.putBoolean(
          "exception thrown for Shooter Extend limit: ",
          isShooterPivotExtException);
    }

    try {
      // This sets a bottom limit for the shooter, and if it fails, throws an error
      ShooterRetLimit = new DigitalInput(Constants.ShooterPivot.DIO_RET_LIMIT);
    } catch (Exception e) {
      isShooterPivotRetException = true;
      SmartDashboard.putBoolean(
          "exception thrown for Shooter Retract limit: ",
          isShooterPivotRetException);
    }
  }

  // methods start here

  /*
  public double getShooterEncoder() {  //gives encoder reading in Revs
    return shooterPivotEncoder.getRaw();
  }

  public void resetShooterEncoder() {
    shooterPivotEncoder.reset();
  }
  */

  public double getShooterEncoder() {
    // getPosition() returns a StatusSignal; .getValueAsDouble() gets the rotation count
    return shooterPivotMotor.getPosition().getValueAsDouble();
  }

  public void resetShooterEncoder() {
    // Sets the current position to 0 rotations
    shooterPivotMotor.setPosition(0);
  }

  public void stopShooterPivot() {
    shooterPivotMotor.stopMotor();
  }

  public double getShooterPivotVelocity() {
    return shooterPivotMotor.getVelocity().getValueAsDouble();
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
    return (getShooterEncoder() >= Constants.ShooterPivot.ENC_REVS_MAX);
  }

  // Replaced DutyCycle speed control with PositionVoltage control
  public void setShooterPivotPosition(double targetRevs) {

    // Clamp target to software limits
    targetRevs =
        Math.max(0.0, Math.min(targetRevs, Constants.ShooterPivot.ENC_REVS_MAX));

    // Prevent driving further into limits
    if (targetRevs > getShooterEncoder() && isShooterExtLimit()) {
      stopShooterPivot();
      return;
    }

    if (targetRevs < getShooterEncoder() && isShooterRetLimit()) {
      stopShooterPivot();
      resetShooterEncoder();
      return;
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
        getShooterEncoder());
    SmartDashboard.putBoolean(
        "Shooter Pivot is fully extended?",
        isFullyExtended());
  }
}
