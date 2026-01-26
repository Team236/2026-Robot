// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Main;

public class FuelShooter extends SubsystemBase {

  private TalonFX leftMainMotor, rightMainMotor;
  private TalonFX leftTopMotor, rightTopMotor;

  private TalonFXConfiguration leftMainConfig, rightMainConfig;
  private TalonFXConfiguration leftTopConfig, rightTopConfig;

  private VelocityVoltage leftMain_m_request;
  private VelocityVoltage leftTop_m_request;

  /** Creates a new FuelShooter. */
  public FuelShooter() {
     leftMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, "usb"); //will be rio bus
     leftTopMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, "usb"); //will be rio bus

        leftMainConfig = new TalonFXConfiguration();
        leftMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        leftMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        leftMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftTopConfig = new TalonFXConfiguration();
        leftTopConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        leftTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftTopConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        leftTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  // set slot 0 gains TODO tune these, find info online (velocity control - no Ks or kA)
  var slot0LMConfigs = leftMainConfig.Slot0;  
    slot0LMConfigs.kV = Constants.Shooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
    slot0LMConfigs.kP = Constants.Shooter.KP_MAIN; //4.8
    slot0LMConfigs.kI = Constants.Shooter.KI_MAIN; 
    slot0LMConfigs.kD = Constants.Shooter.KD_MAIN;

  var slot0LTConfigs = leftTopConfig.Slot0;  
    slot0LTConfigs.kV = Constants.Shooter.KV_TOP; // FF. A velocity target of 1 rps results in 0.12 V output
    slot0LTConfigs.kP = Constants.Shooter.KP_TOP; //4.8
    slot0LTConfigs.kI = Constants.Shooter.KI_TOP; 
    slot0LTConfigs.kD = Constants.Shooter.KD_TOP;

    leftMainMotor.getConfigurator().apply(leftMainConfig);
    leftTopMotor.getConfigurator().apply(leftTopConfig);

  leftMain_m_request = new VelocityVoltage(0).withSlot(0);
  leftTop_m_request = new VelocityVoltage(0).withSlot(0);

    //MAKE RIGHT FOLLOW LEFT 
    rightMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_MAIN, "usb"); //will be rio bus
    rightTopMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_TOP, "usb"); //will be rio bus

        rightMainConfig = new TalonFXConfiguration();
        rightMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        rightMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        rightMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightTopConfig = new TalonFXConfiguration();
        rightTopConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        rightTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightTopConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        rightTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  // set slot 0 gains TODO tune these, find info online
  var slot0RMConfigs = rightMainConfig.Slot0;  
    slot0RMConfigs.kV = Constants.Shooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
    slot0RMConfigs.kP = Constants.Shooter.KP_MAIN;//4.8
    slot0RMConfigs.kI = Constants.Shooter.KI_MAIN; // no output for integrated error
    slot0RMConfigs.kD = Constants.Shooter.KD_MAIN;

  var slot0RTConfigs = rightTopConfig.Slot0;  
    slot0RTConfigs.kV = Constants.Shooter.KV_TOP; // FF. A velocity target of 1 rps results in 0.12 V output
    slot0RTConfigs.kP = Constants.Shooter.KP_TOP;//4.8
    slot0RTConfigs.kI = Constants.Shooter.KI_TOP; // no output for integrated error
    slot0RTConfigs.kD = Constants.Shooter.KD_TOP;

    rightMainMotor.getConfigurator().apply(rightMainConfig);
    rightTopMotor.getConfigurator().apply(rightTopConfig);

    rightMainMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, MotorAlignmentValue.Opposed));
    rightTopMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, MotorAlignmentValue.Opposed));
  }

  // METHODS START HERE:

  public void shootFuel(double targetMainVelocity, double targetTopVelocity) {
   //rightMainMotor and rightTopMotor will follow their corresponding left motor
   //the target velocity below needs to be in revs per second
    leftMainMotor.setControl(leftMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.Shooter.KV_MAIN));
    leftTopMotor.setControl(leftTop_m_request.withVelocity(targetTopVelocity).withFeedForward(Constants.Shooter.KV_TOP));
  }

  public void spinMainMotor (double manualMainSpeed) {
    leftMainMotor.set(manualMainSpeed); // between -1 and 1
  }

  // between -1 and 1
  public void spinTopMotor (double manualTopSpeed) {
    leftTopMotor.set(manualTopSpeed);
  }

  // this is motor speed between -1.0 and 1.0
  public double getLeftSpeed() {
    return leftMainMotor.get();
  }

  public double getLeftSpeedTop() {
    return leftTopMotor.get();
  }

  // this is motor speed between -1.0 and 1.0
  public double getRightSpeed() {
    return rightMainMotor.get();
  }

  public double getRightSpeedTop() {
    return rightTopMotor.get();
  }

  // this is motor speed between -1.0 and 1.0
  public double getAvgSpeed() {
    return (rightMainMotor.get() + leftMainMotor.get()) / 2;
  }

  // this value is in RPS, rotations per second between -512 to 512
  public double getLeftVelocity() {
    return leftMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getLeftVelocityTop() {
    return leftTopMotor.getRotorVelocity().getValueAsDouble();
  }

  // this value is in RPS, rotations per second
  public double getRightVelocity() {
    return rightMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getRightVelocityTop() {
    return rightTopMotor.getRotorVelocity().getValueAsDouble();
  }

  public void stopShooter() {
    leftMainMotor.set(0);
    leftTopMotor.set(0);
    // rightElevatorMotor.set(0); // Right is follower, so it also stops
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left main velocity", getLeftVelocity());
    SmartDashboard.putNumber("left top velocity", getLeftVelocityTop());
    SmartDashboard.putNumber("right main velocity", getRightVelocity());
    SmartDashboard.putNumber("right top velocity", getRightVelocityTop());
  }
}