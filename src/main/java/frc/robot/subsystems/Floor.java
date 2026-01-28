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

public class Floor extends SubsystemBase {

  private TalonFX floorMotor;
  private TalonFXConfiguration motorConfig;

  /** Creates a new Floor. */
  //This system is a motor that moves the fuel from the intake into the bin, toward the shooter feeder
  // at a constant speed (and can eject fuel from the bin onto the floor at a set speed)
  public Floor() {
    floorMotor = new TalonFX(Constants.MotorControllers.ID_FLOOR, "usb"); //will be rio bus
    
    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    floorMotor.getConfigurator().apply(motorConfig);
  }

  //METHODS START HERE:

  public double getFloorSpeed() {
    return floorMotor.get();
  }

  public void setFloorSpeed(double speed) {
    floorMotor.set(speed);
  }

  public void stopFloor() {
    floorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Floor speed", getFloorSpeed());
  }
}
