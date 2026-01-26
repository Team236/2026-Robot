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

public class Feeder extends SubsystemBase {

  private TalonFX feederMotor;
  private TalonFXConfiguration motorConfig;

  /** Creates a new Feeder. */
  public Feeder() {
    feederMotor = new TalonFX(Constants.MotorControllers.ID_FEEDER, "usb"); //will be rio bus
    
    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederMotor.getConfigurator().apply(motorConfig);
  }

  //METHODS START HERE:

  public double getFeederSpeed() {
    return feederMotor.get();
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void stopFeeder() {
    feederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Feeder speed", getFeederSpeed());
  }
}
