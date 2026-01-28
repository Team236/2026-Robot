// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PreFeeder extends SubsystemBase {
  
  private TalonFX preFeederMotor;
  private TalonFXConfiguration motorConfig;

  public PreFeeder() {
    preFeederMotor = new TalonFX(Constants.MotorControllers.ID_PRE_FEEDER, "usb");
    
    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    preFeederMotor.getConfigurator().apply(motorConfig);
  }

  public double getPreFeederSpeed() {
    return preFeederMotor.get();
  }

  public void setPreFeederSpeed(double speed) {
    preFeederMotor.set(speed);
  }

  public void stopPreFeeder() {
    preFeederMotor.set(0);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("PreFeeder speed:", getPreFeederSpeed());
  }
}
