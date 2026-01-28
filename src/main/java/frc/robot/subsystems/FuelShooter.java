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

  private TalonFX leftMainMotor, rightMainMotor, midMainMotor;
  private TalonFX leftTopMotor, rightTopMotor;

  private TalonFXConfiguration leftMainConfig, rightMainConfig, midMainConfig;
  private TalonFXConfiguration leftTopConfig, rightTopConfig;

  private VelocityVoltage leftMain_m_request, rightMain_m_request, midMain_m_request;
  private VelocityVoltage leftTop_m_request, rightTop_m_request;

  /** Creates a new FuelShooter. */
  //This system uses motors to shoot the fuel with a constant velocity which is 
  //quickly brought up to speed and maintains that speed using PID velocity control
  public FuelShooter() {
  //MAKE EACH MOTOR INDEPENDENT - NOT FOLLOWERS (commented out Follower code far below)
  
     leftMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, "usb"); //will be rio bus
        leftMainConfig = new TalonFXConfiguration();
        leftMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        leftMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        leftMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // set slot 0 gains TODO tune these, find info online (velocity control - no Ks or kA)
      var slot0LMConfigs = leftMainConfig.Slot0;  
        slot0LMConfigs.kV = Constants.Shooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0LMConfigs.kP = Constants.Shooter.KP_MAIN; //4.8
        slot0LMConfigs.kI = Constants.Shooter.KI_MAIN; 
        slot0LMConfigs.kD = Constants.Shooter.KD_MAIN;

      leftMainMotor.getConfigurator().apply(leftMainConfig);
      leftMain_m_request = new VelocityVoltage(0).withSlot(0);


    rightMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_MAIN, "usb"); //will be rio bus
        rightMainConfig = new TalonFXConfiguration();
        rightMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        rightMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        rightMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // set slot 0 gains TODO tune these, find info online
      var slot0RMConfigs = rightMainConfig.Slot0;  
        slot0RMConfigs.kV = Constants.Shooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0RMConfigs.kP = Constants.Shooter.KP_MAIN;//4.8
        slot0RMConfigs.kI = Constants.Shooter.KI_MAIN; // no output for integrated error
        slot0RMConfigs.kD = Constants.Shooter.KD_MAIN;

      rightMainMotor.getConfigurator().apply(rightMainConfig);
      rightMain_m_request = new VelocityVoltage(0).withSlot(0);
      //rightMainMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, MotorAlignmentValue.Opposed));


    midMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_MID_MAIN, "usb"); //will be rio bus
        midMainConfig = new TalonFXConfiguration();
        midMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        midMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        midMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        midMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // set slot 0 gains TODO tune these, find info online (velocity control - no Ks or kA)
      var slot0MMConfigs = midMainConfig.Slot0;  
        slot0MMConfigs.kV = Constants.Shooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0MMConfigs.kP = Constants.Shooter.KP_MAIN; //4.8
        slot0MMConfigs.kI = Constants.Shooter.KI_MAIN; 
        slot0MMConfigs.kD = Constants.Shooter.KD_MAIN;

      midMainMotor.getConfigurator().apply(leftMainConfig);
      midMain_m_request = new VelocityVoltage(0).withSlot(0);
      //midMainMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, MotorAlignmentValue.Opposed));


    leftTopMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, "usb"); //will be rio bus
        leftTopConfig = new TalonFXConfiguration();
        leftTopConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        leftTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftTopConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        leftTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // set slot 0 gains TODO tune these, find info online (velocity control - no Ks or kA) 
      var slot0LTConfigs = leftTopConfig.Slot0;  
        slot0LTConfigs.kV = Constants.Shooter.KV_TOP; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0LTConfigs.kP = Constants.Shooter.KP_TOP; //4.8
        slot0LTConfigs.kI = Constants.Shooter.KI_TOP; 
        slot0LTConfigs.kD = Constants.Shooter.KD_TOP;

      leftTopMotor.getConfigurator().apply(leftTopConfig);
      leftTop_m_request = new VelocityVoltage(0).withSlot(0);


    rightTopMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_TOP, "usb"); //will be rio bus
        rightTopConfig = new TalonFXConfiguration();
        rightTopConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        rightTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightTopConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        rightTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // set slot 0 gains TODO tune these, find info online
      var slot0RTConfigs = rightTopConfig.Slot0;  
        slot0RTConfigs.kV = Constants.Shooter.KV_TOP; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0RTConfigs.kP = Constants.Shooter.KP_TOP;//4.8
        slot0RTConfigs.kI = Constants.Shooter.KI_TOP; // no output for integrated error
        slot0RTConfigs.kD = Constants.Shooter.KD_TOP;

      rightTopMotor.getConfigurator().apply(rightTopConfig);
      rightTop_m_request = new VelocityVoltage(0).withSlot(0);
    // rightTopMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, MotorAlignmentValue.Opposed));
  }

  // METHODS START HERE:

  public void lefMainPID(double targetMainVelocity) {//the target velocity below needs to be in revs per second
    leftMainMotor.setControl(leftMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.Shooter.KV_MAIN));
  }
  
  public void rightMainPID(double targetMainVelocity) { //the target velocity below needs to be in revs per second
    rightMainMotor.setControl(rightMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.Shooter.KV_MAIN));
  }
  
  public void midMainPID(double targetMainVelocity) {//the target velocity below needs to be in revs per second
    midMainMotor.setControl(midMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.Shooter.KV_MAIN));
  }
  
  public void leftTopPID(double targetTopVelocity) {//the target velocity below needs to be in revs per second
    leftTopMotor.setControl(leftTop_m_request.withVelocity(targetTopVelocity).withFeedForward(Constants.Shooter.KV_TOP));
  }
  
  public void rightTopPID(double targetTopVelocity) { //the target velocity below needs to be in revs per second
    rightTopMotor.setControl(rightTop_m_request.withVelocity(targetTopVelocity).withFeedForward(Constants.Shooter.KV_TOP));
  }

  public void spinLMainMotor (double manualMainSpeed) {
    leftMainMotor.set(manualMainSpeed); // between -1 and 1
  }
public void spinRMainMotor (double manualMainSpeed) {
    rightMainMotor.set(manualMainSpeed); // between -1 and 1
  }
  public void spinMidMainMotor (double manualMainSpeed) {
    midMainMotor.set(manualMainSpeed); // between -1 and 1
  }

  // between -1 and 1
  public void spinTopLeftMotor (double manualTopSpeed) {
    leftTopMotor.set(manualTopSpeed);
  }

   // between -1 and 1
  public void spinTopRightMotor (double manualTopSpeed) {
    rightTopMotor.set(manualTopSpeed);
  }

  // this is motor speed between -1.0 and 1.0
  public double getLeftMainSpeed() {
    return leftMainMotor.get();
  }

  public double getLeftTopSpeed() {
    return leftTopMotor.get();
  }

  // this is motor speed between -1.0 and 1.0
  public double getRightMainSpeed() {
    return rightMainMotor.get();
  }

  public double getRightTopSpeed() {
    return rightTopMotor.get();
  }

  // this is motor speed between -1.0 and 1.0
  public double getAvgMainSpeed() {
    return (rightMainMotor.get() + leftMainMotor.get() + midMainMotor.get()) / 3;
  }

    // this is motor speed between -1.0 and 1.0
  public double getAvgTopSpeed() {
    return (rightTopMotor.get() + leftTopMotor.get()) / 2;
  }

  // this value is in RPS, rotations per second between -512 to 512
  public double getLeftMainVelocity() {
    return leftMainMotor.getRotorVelocity().getValueAsDouble();
  }

  // this value is in RPS, rotations per second
  public double getRightMainVelocity() {
    return rightMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getMidMainVelocity() {
    return midMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getLeftTopVelocity() {
    return leftTopMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getRightTopVelocity() {
    return rightTopMotor.getRotorVelocity().getValueAsDouble();
  }

  public void stopShooter() {
    leftMainMotor.stopMotor();
    rightMainMotor.stopMotor();
    midMainMotor.stopMotor();
    leftTopMotor.stopMotor();
    rightTopMotor.stopMotor();
  }
    
  public void stopLeftMain(){
      leftMainMotor.stopMotor();
    }
  public void stopRightMain(){
      rightMainMotor.stopMotor();
    }  
    public void stopMidMain(){
      midMainMotor.stopMotor();
    }  
    public void stopLeftTop(){
      leftTopMotor.stopMotor();
    }  
    public void stopRightTop(){
      rightTopMotor.stopMotor();
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left main velocity: ", getLeftMainVelocity());
    SmartDashboard.putNumber("right main velocity: ", getRightMainVelocity());
    SmartDashboard.putNumber("middle main velocity: ", getMidMainVelocity());
   
    SmartDashboard.putNumber("left top velocity: ", getLeftTopVelocity());
    SmartDashboard.putNumber("right top velocity: ", getRightTopVelocity());

  }
}