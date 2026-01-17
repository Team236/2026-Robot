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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class FuelShooter extends SubsystemBase {

private TalonFX leftMotor, rightMotor;
private TalonFXConfigurator leftConfig, rightConfig;
private TalonFXConfiguration leftTalonConfig, rightTalonConfig;

private VelocityVoltage Leftm_request, Rightm_request;


  /** Creates a new FuelShooter. */
  public FuelShooter() {

    // TODO make sure this controller is on usb bus
     leftMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_TALON, "usb");

    // configure motors
    leftTalonConfig = new TalonFXConfiguration();
    leftTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    leftTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  // set slot 0 gains TODO tune these, find info online
    var slot0LConfigs =  leftTalonConfig.Slot0;
   // var slot0LConfigs = new Slot0Configs();
   // slot0LConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0LConfigs.kV = Constants.FuelShooter.KV; // FF. A velocity target of 1 rps results in 0.12 V output
    // slot0LConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0LConfigs.kP = Constants.FuelShooter.KP;//4.8
    slot0LConfigs.kI = Constants.FuelShooter.KI; // no output for integrated error
    slot0LConfigs.kD = Constants.FuelShooter.KD;

    // TODO Find actual values
   // leftMotor.getConfigurator().apply (slot0LConfigs);
      leftMotor.getConfigurator().apply (leftTalonConfig);
    final VelocityVoltage Leftm_request = new VelocityVoltage(0).withSlot(0);
    // TODO Do this in a method
    


    /* set Motion Magic settings
    var motionLMagicConfigs = leftTalonConfig.MotionMagic;
    motionLMagicConfigs.MotionMagicCruiseVelocity = 80;//80; // Target cruise velocity of 80 rps
    motionLMagicConfigs.MotionMagicAcceleration = 120;//160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionLMagicConfigs.MotionMagicJerk = 1200;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leftMotor.getConfigurator().apply(leftTalonConfig);
*/
     
    rightMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_TALON, "usb");

    rightTalonConfig = new TalonFXConfiguration();
    //rightTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    rightTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

     // set slot 0 gains
     var slot0RConfigs =  rightTalonConfig.Slot0;
   // var slot0RConfigs = rightTalonConfig.Slot0;
   // slotRConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0RConfigs.kV = Constants.FuelShooter.KV; // A velocity target of 1 rps results in 0.12 V output
   // slotRConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0RConfigs.kP = Constants.FuelShooter.KP;//4.8; //START LOWER??// A position error of 2.5 rotations results in 12 V output
    slot0RConfigs.kI = Constants.FuelShooter.KI; // no output for integrated error
    slot0RConfigs.kD = Constants.FuelShooter.KD; // no output for derivative error

    rightMotor.getConfigurator().apply (rightTalonConfig);
    // rightMotor.getConfigurator().apply (slot0RConfigs);
    final VelocityVoltage Rightm_request = new VelocityVoltage(0).withSlot(0);

  }

    /* set Motion Magic settings
    var motionRMagicConfigs = rightTalonConfig.MotionMagic;
    motionRMagicConfigs.MotionMagicCruiseVelocity = 80;//80; // Target cruise velocity of 80 rps
    motionRMagicConfigs.MotionMagicAcceleration = 120;//160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionRMagicConfigs.MotionMagicJerk = 1200;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    rightMotor.getConfigurator().apply(rightTalonConfig);

    rightMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_TALON, false));

    m_request = new MotionMagicVoltage(0);
    */

    // Methods Start Here

    public void shootFuel (double targetVelocity, double feedForward) {
     rightMotor.setControl(Rightm_request.withVelocity(targetVelocity).withFeedForward(feedForward));
     leftMotor.setControl(Leftm_request.withVelocity(targetVelocity).withFeedForward(feedForward));

      //rightMotor.setControl(Rightm_request.withVelocity(targetVelocity));
     // leftMotor.setControl(Leftm_request.withVelocity(targetVelocity));
    }


    public double getLeftSpeed() {
    return leftMotor.get();
  }  

  public double getRightSpeed() {
    return rightMotor.get();
  }  

  public double getAvgSpeed() {
    return (rightMotor.get() + leftMotor.get()) / 2;
  }

  // this value is in RPS, rotations per second between -512 to 512
  public double getLeftVelocity() {
    return leftMotor.getRotorVelocity().getValueAsDouble();
  }


  public void stopShooter() {
    leftMotor.set(0);
   // rightElevatorMotor.set(0); // Right is follower, so it also stops
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     
}

}
