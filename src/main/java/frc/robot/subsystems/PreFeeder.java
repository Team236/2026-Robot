// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PreFeeder extends SubsystemBase {
  
  private TalonFX preFeederMotor;
  private TalonFXConfiguration motorConfig;

  public static Counter counter;
  public boolean isCounterUnplugged = false;
  public boolean isSensorUnplugged = false;
  public DigitalInput lightSensorState;

  public PreFeeder() {
    preFeederMotor = new TalonFX(Constants.MotorControllers.ID_PRE_FEEDER, "usb");
    
    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    preFeederMotor.getConfigurator().apply(motorConfig);

    try {
      lightSensorState = new DigitalInput(Constants.CoralHold.DIO_COUNTER);
    } catch (Exception e)
    {
      isSensorUnplugged = true;
      SmartDashboard.putBoolean("is lightSensor unplugged:", isSensorUnplugged);
    }


    try {
      counter = new Counter();
      counter.setUpSource(Constants.CoralHold.DIO_COUNTER);
      counter.reset();
    }
    catch (Exception e) {
      isCounterUnplugged = true;
    }

    SmartDashboard.putBoolean("is counter unplugged:", isCounterUnplugged);
    SmartDashboard.putBoolean("is sensor unplugged:", isSensorUnplugged);
    counter.reset(); //sets counter to zero
  }

  public int getHCount() {
    int count;
    if (isCounterUnplugged) {
      count = 0;
      SmartDashboard.putBoolean("Intake counter unplugged:", isCounterUnplugged);
    } else {
      count =  counter.get();
    }
    return count;
  }

  public boolean getLightSensorState() {
  boolean sensorState;
    if (isSensorUnplugged) {
      sensorState = false;
      SmartDashboard.putBoolean("LightSensor is unplugged", isCounterUnplugged);
    } else {
      sensorState =  lightSensorState.get();
    }
    return sensorState;
  }

  public void resetCount() {
    // automaticaly sets counter to 0 at start 
    counter.reset();
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
    
    SmartDashboard.putBoolean("Coral Light Sensor State is:", getLightSensorState());
    SmartDashboard.putNumber("PreFeeder speed:", getPreFeederSpeed());
  }
}
