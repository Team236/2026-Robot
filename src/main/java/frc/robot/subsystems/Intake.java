// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotorLeft, intakeMotorRight;
  private TalonFXConfigurator config;
  private TalonFXConfiguration talonConfigLeft, talonConfigRight;
  private CurrentLimitsConfigs currentConfigs;
  private MotorOutputConfigs outputConfigs;

  /** Creates a new Intake. */
  //This system uses a motor to intake in , or spit out, fuel at a constant speed
  public Intake() {

    // Left motor
    intakeMotorLeft = new TalonFX(Constants.MotorControllers.ID_INTAKE_LEFT, "usb");
    talonConfigLeft = new TalonFXConfiguration();
    // Motor Output Configs
    talonConfigLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfigLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfigLeft.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    talonConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Right motor
    intakeMotorRight = new TalonFX(Constants.MotorControllers.ID_INTAKE_RIGHT, "usb");
    talonConfigRight = new TalonFXConfiguration();
    // Motor Output Configs
    talonConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfigRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfigRight.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    talonConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotorLeft.getConfigurator().apply(talonConfigRight);

    intakeMotorRight.setControl(new Follower(Constants.MotorControllers.ID_INTAKE_LEFT, MotorAlignmentValue.Opposed));
  }

  public void intakeStop() {
    intakeMotorLeft.set(0);
  }

  public void intakeIn(double speed) {
    intakeMotorLeft.set(speed);
  }

  public void intakeOut(double speed) {
    intakeMotorLeft.set(speed);
  }

  public double getIntakeSpeed() {
    return intakeMotorLeft.get();
  }

  @Override
  public void periodic() {     
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake speed", getIntakeSpeed());

  }
}
