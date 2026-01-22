// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private TalonFX intakeMotor;
  private TalonFXConfigurator config;
  private TalonFXConfiguration talonConfig;

  private CurrentLimitsConfigs currentConfigs;
  private MotorOutputConfigs outputConfigs;


  public Intake() {
    // im bad at coding 
    intakeMotor = new TalonFX(Constants.FloorIntake.INTAKE_MOTOR_ID, "usb");

    config = new TalonFXConfigurator(null);
    talonConfig = new TalonFXConfiguration();

    // Motor Output Configs
    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.CurrentLimits = currentConfigs;

    config.apply(talonConfig);

  }

  public void intakeStop() {
    intakeMotor.set(0);
  }

  public void intakeIn() {
    intakeMotor.set(Constants.FloorIntake.INTAKE_SPEED);
  }

  public void intakeOut() {
    intakeMotor.set(Constants.FloorIntake.OUTTAKE_SPEED);
  }

  public double getIntakeSpeed() {
    return intakeMotor.get();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
