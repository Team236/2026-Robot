// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TopRoller extends SubsystemBase {

  private TalonFX leftTopMotor, rightTopMotor;

  private TalonFXConfiguration leftTopConfig, rightTopConfig;

  private VelocityVoltage leftTop_m_request, rightTop_m_request;

  /** Creates a new TopRoller. */
  //This system uses motors to shoot the fuel with a constant velocity which is 
  //quickly brought up to speed and maintains that speed using PID velocity control
  public TopRoller() {

  //MAKE RIGHT MOTOR BE A FOLLOWER OF THE LEFT
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

      rightTopMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, MotorAlignmentValue.Opposed));
  }

  // METHODS START HERE:
  public void TopPID(double targetTopVelocity) {//the target velocity below needs to be in revs per second
    leftTopMotor.setControl(leftTop_m_request.withVelocity(targetTopVelocity).withFeedForward(Constants.Shooter.KV_TOP));
  }
  
  // between -1 and 1
  public void spinTopMotor (double manualTopSpeed) {
    leftTopMotor.set(manualTopSpeed);
  }

  public double getTopSpeed() {
    return leftTopMotor.get();
  }

  public double getTopVelocity() {
    return leftTopMotor.getRotorVelocity().getValueAsDouble();
  }

  public void stopTop(){
      leftTopMotor.stopMotor();
    } 

 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("top velocity: ", getTopVelocity());
  }
}