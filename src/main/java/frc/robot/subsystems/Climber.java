// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX climbMotor;
  private TalonFXConfiguration motorConfig;
  private MotionMagicVoltage m_request;

  private boolean isTopException = false;
  private boolean isBottomException = false;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  /** Creates a new Climber. */
  //This system uses a motor to lift the robot off the floor (climb), using MotionMagic position control
  public Climber() {
    // MOTOR
    climbMotor = new TalonFX(Constants.MotorControllers.ID_CLIMBER, "usb"); //will be rio bus

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0Configs = motorConfig.Slot0;
    // TUNE THESE -- COPIED FROM 2025 FOR NOW
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 2;//4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
    // TUNE THESE -- COPIED FROM 2025 FOR NOW
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;//80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 120;//160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1200;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    climbMotor.getConfigurator().apply(motorConfig);

    m_request = new MotionMagicVoltage(0);

    // LIMIT SWITCHES
    try {
      topLimitSwitch = new DigitalInput(Constants.Climb.DIO_CLIMBER_TOP);
    } catch (Exception e) {
      isTopException = true;
    }
    
    try {
      bottomLimitSwitch = new DigitalInput(Constants.Climb.DIO_CLIMBER_BOTTOM);
    } catch (Exception e) {
      isBottomException = true;
    }
  }

  //METHODS START HERE:

  public boolean isTopLimit() {
    if (isTopException) {
      return true;
    }
    return topLimitSwitch.get() || getClimberEncoder() > Constants.Climb.MAX_ENCODER_REVS;
  }

  public boolean isBottomLimit() {
    if (isBottomException) {
      return true;
    }
    return bottomLimitSwitch.get() || getClimberEncoder() < 0;
  }

  public double getClimberEncoder() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  public void resetClimberEncoder() {
    climbMotor.setPosition(0);
  }

  public void stopClimber() {
    climbMotor.set(0);
  }

  public void setClimberSpeed(double speed) {
    if (speed <= 0 && isBottomLimit()) {
      resetClimberEncoder();
      stopClimber();
    } else if (speed > 0 && isTopLimit()) {
      stopClimber();

    } else {
      setClimberSpeedOverride(speed);
    }
  }

  public void setClimberSpeedOverride(double speed) {
    climbMotor.set(speed);
  }

  public double getClimberSpeed() {
    return climbMotor.get();
  }

  // TODO: ideally take in desiredHeight or desiredInches once revs to inches conversion is known
  public void doMotionMagic(double desiredRevs) {
    if (climbMotor.get() > 0 && isTopLimit()) {
      stopClimber();
      return;
    } else if (climbMotor.get() < 0 && isBottomLimit()) {
      resetClimberEncoder();
      stopClimber();
    } else {
      climbMotor.setControl(m_request.withPosition(desiredRevs));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber encoder revs", getClimberEncoder());
    SmartDashboard.putNumber("Climber motor speed", getClimberSpeed());
    SmartDashboard.putBoolean("Climber top limit EXCEPTION", isTopException);
    SmartDashboard.putBoolean("Climber bottom limit EXCEPTION", isBottomException);
    SmartDashboard.putBoolean("Climber top limit hit", isTopLimit());
    SmartDashboard.putBoolean("Climber bottom limit hit", isBottomLimit());
  }
}
