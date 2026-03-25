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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotor, intakeMotorRight;
  private TalonFXConfigurator config;
  private TalonFXConfiguration talonConfig, talonConfigRight;
  private CurrentLimitsConfigs currentConfigs;
  private MotorOutputConfigs outputConfigs;
  private VelocityVoltage intake_m_request;

  /** Creates a new Intake. */
  //This system uses a motor to intake in , or spit out, fuel at a constant speed
  public Intake() {

    // Left motor
    intakeMotor = new TalonFX(Constants.MotorControllers.ID_INTAKE_LEFT, "rio");
    talonConfig = new TalonFXConfiguration();
    // Motor Output Configs
    talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.INTAKE_CURRENT_LIMIT;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0RMConfigs = talonConfig.Slot0;  
      slot0RMConfigs.kV = Constants.IntakeConstants.KV_I;
      slot0RMConfigs.kP = Constants.IntakeConstants.KP_I;
      slot0RMConfigs.kI = Constants.IntakeConstants.KI_I;
      slot0RMConfigs.kD = Constants.IntakeConstants.KD_I;

      intakeMotor.getConfigurator().apply(talonConfig);
      intake_m_request = new VelocityVoltage(0).withSlot(0);

    // // Right motor
    // intakeMotorRight = new TalonFX(Constants.MotorControllers.ID_INTAKE_RIGHT, "usb");
    // talonConfigRight = new TalonFXConfiguration();
    // // Motor Output Configs
    // talonConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // talonConfigRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    // talonConfigRight.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    // talonConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // intakeMotorLeft.getConfigurator().apply(talonConfigRight);

    // intakeMotorRight.setControl(new Follower(Constants.MotorControllers.ID_INTAKE_LEFT, MotorAlignmentValue.Opposed));
  }

  public void IntakePID(double targetMainVelocity) {//the target velocity must be in revs per second
    double motorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble(); // Amps

    double motorRPS = intakeMotor.getVelocity().getValueAsDouble();
    double motorRPM = motorRPS * 60.0; 

    double STALL_CURRENT = 18.0; // Amps
    double STALL_RPM = 750.0; 

    if (motorCurrent >= STALL_CURRENT && Math.abs(motorRPM) < STALL_RPM) {
      RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else {
        RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    intakeMotor.setControl(intake_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.IntakeConstants.KV_I));
  }

  public void intakeStop() {
    intakeMotor.stopMotor();
  }

  public void intakeIn(double speed) {
    intakeMotor.set(speed);
  }

  public void intakeOut(double speed) {
    intakeMotor.set(speed);
  }

  public double getIntakeSpeed() {
    return intakeMotor.get();
  }

  @Override
  public void periodic() {     
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake speed", getIntakeSpeed());

  }
}
