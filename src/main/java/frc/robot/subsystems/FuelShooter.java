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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Main;

public class FuelShooter extends SubsystemBase {

  private TalonFX leftMainMotor, rightMainMotor;
  private TalonFX leftTopMotor, rightTopMotor;

  private TalonFXConfiguration leftMainConfig, rightMainConfig;
  private TalonFXConfiguration leftTopConfig, rightTopConfig;

  private VelocityVoltage leftMain_m_request, rightMain_m_request;
  private VelocityVoltage leftTop_m_request, rightTop_m_request;
  /** Creates a new FuelShooter. */
  public FuelShooter() {

    // TODO make sure this controller is on usb bus
     leftMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, "usb");
     leftTopMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, "usb");

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

  // set slot 0 gains TODO tune these, find info online
  var slot0LMConfigs = leftMainConfig.Slot0;  
    //slot0LMConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0LMConfigs.kV = Constants.FuelShooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
    //slot0LMConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0LMConfigs.kP = Constants.FuelShooter.KP_MAIN; //4.8
    slot0LMConfigs.kI = Constants.FuelShooter.KI_MAIN; // no output for integrated error
    slot0LMConfigs.kD = Constants.FuelShooter.KD_MAIN;

  var slot0LTConfigs = leftTopConfig.Slot0;  
    //slot0LTConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0LTConfigs.kV = Constants.FuelShooter.KV_TOP; // FF. A velocity target of 1 rps results in 0.12 V output
    //slot0LTConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0LTConfigs.kP = Constants.FuelShooter.KP_TOP; //4.8
    slot0LTConfigs.kI = Constants.FuelShooter.KI_TOP; // no output for integrated error
    slot0LTConfigs.kD = Constants.FuelShooter.KD_TOP;


  leftMainMotor.getConfigurator().apply(leftMainConfig);
  leftTopMotor.getConfigurator().apply(leftTopConfig);

   leftMain_m_request = new VelocityVoltage(0).withSlot(0);
   leftTop_m_request = new VelocityVoltage(0).withSlot(0);

    //MAKE RIGHT FOLLOW LEFT 
      rightMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_MAIN, "usb");
      rightTopMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_TOP, "usb");

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
   //slot0RMConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0RMConfigs.kV = Constants.FuelShooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
    // slot0RMConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0RMConfigs.kP = Constants.FuelShooter.KP_MAIN;//4.8
    slot0RMConfigs.kI = Constants.FuelShooter.KI_MAIN; // no output for integrated error
    slot0RMConfigs.kD = Constants.FuelShooter.KD_MAIN;

  var slot0RTConfigs = rightTopConfig.Slot0;  
   //slot0RTConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0RTConfigs.kV = Constants.FuelShooter.KV_TOP; // FF. A velocity target of 1 rps results in 0.12 V output
    // slot0RTConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0RTConfigs.kP = Constants.FuelShooter.KP_TOP;//4.8
    slot0RTConfigs.kI = Constants.FuelShooter.KI_TOP; // no output for integrated error
    slot0RTConfigs.kD = Constants.FuelShooter.KD_TOP;


    rightMainMotor.getConfigurator().apply(rightMainConfig);
    rightTopMotor.getConfigurator().apply(rightTopConfig);

    rightMainMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, MotorAlignmentValue.Opposed));
    rightTopMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_TOP, MotorAlignmentValue.Opposed));
  }

  // Methods Start Here
  public void shootFuel(double targetMianVelocity, double targetTopVelocity, double feedForwardMain, double feedForwardTop) {
   //rightMainMotor and rightTopMotor will follow their corresponding left motor
    leftMainMotor.setControl(leftMain_m_request.withVelocity(targetMianVelocity).withFeedForward(feedForwardMain));
    leftTopMotor.setControl(leftTop_m_request.withVelocity(targetTopVelocity).withFeedForward(feedForwardTop));
    // rightMainMotor.setControl(Rightm_request.withVelocity(targetVelocity));
    // leftMainMotor.setControl(Main.withVelocity(targetVelocity));
  }

  // this is motor speed between -1.0 and 1.0
  public double getLeftSpeed() {
    return leftMainMotor.get();
  }

  // this is motor speed between -1.0 and 1.0
  public double getRightSpeed() {
    return rightMainMotor.get();
  }

  // this is motor speed between -1.0 and 1.0
  public double getAvgSpeed() {
    return (rightMainMotor.get() + leftMainMotor.get()) / 2;
  }

  // this value is in RPS, rotations per second between -512 to 512
  public double getLeftVelocity() {
    return leftMainMotor.getRotorVelocity().getValueAsDouble();
  }

  // this value is in RPS, rotations per second
  public double getRightVelocity() {
    return rightMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public void stopShooter() {
    leftMainMotor.set(0);
    // rightElevatorMotor.set(0); // Right is follower, so it also stops
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}