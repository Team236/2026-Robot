// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MainRoller extends SubsystemBase {

  private TalonFX leftMainMotor, rightMainMotor; //, midMainMotor;

  private TalonFXConfiguration leftMainConfig, rightMainConfig; //, midMainConfig;

  private VelocityVoltage leftMain_m_request, rightMain_m_request;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();


  /** Creates a new MainRoller. */
  //This system uses motors to shoot the fuel with a constant velocity which is 
  //quickly brought up to speed and maintains that speed using PID velocity control
  public MainRoller() {

  //MAKE RIGHT MOTOR FOLLOW LEFT
     leftMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, "usb"); //will be rio bus
        leftMainConfig = new TalonFXConfiguration();
        leftMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        leftMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        leftMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // set slot 0 gains TODO tune these, find info online (velocity control - no Ks or kA)
      var slot0LMConfigs = leftMainConfig.Slot0;  
        slot0LMConfigs.kV = Constants.ShooterConstants.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0LMConfigs.kP = Constants.ShooterConstants.KP_MAIN; //4.8
        slot0LMConfigs.kI = Constants.ShooterConstants.KI_MAIN; 
        slot0LMConfigs.kD = Constants.ShooterConstants.KD_MAIN;

      leftMainMotor.getConfigurator().apply(leftMainConfig);
      leftMain_m_request = new VelocityVoltage(0).withSlot(0);


    rightMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_MAIN, "usb"); //will be rio bus
        rightMainConfig = new TalonFXConfiguration();
        rightMainConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //tbd
        rightMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        rightMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // set slot 0 gains TODO tune these, find info online
      var slot0RMConfigs = rightMainConfig.Slot0;  
        slot0RMConfigs.kV = Constants.ShooterConstants.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0RMConfigs.kP = Constants.ShooterConstants.KP_MAIN;//4.8
        slot0RMConfigs.kI = Constants.ShooterConstants.KI_MAIN; // no output for integrated error
        slot0RMConfigs.kD = Constants.ShooterConstants.KD_MAIN;

      rightMainMotor.getConfigurator().apply(rightMainConfig);
      rightMain_m_request = new VelocityVoltage(0).withSlot(0);
      // rightMainMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, MotorAlignmentValue.Opposed));

    /*midMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_MID_MAIN, "usb"); //will be rio bus
        midMainConfig = new TalonFXConfiguration();
        midMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //tbd
        midMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        midMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT; //tbd
        midMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // set slot 0 gains TODO tune these, find info online (velocity control - no Ks or kA)
      var slot0MMConfigs = midMainConfig.Slot0;  
        slot0MMConfigs.kV = Constants.Shooter.KV_MAIN; // FF. A velocity target of 1 rps results in 0.12 V output
        slot0MMConfigs.kP = Constants.Shooter.KP_MAIN; //4.8
        slot0MMConfigs.kI = Constants.Shooter.KI_MAIN; 
        slot0MMConfigs.kD = Constants.Shooter.KD_MAIN;

      midMainMotor.getConfigurator().apply(midMainConfig);
      midMainMotor.setControl(new Follower(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, MotorAlignmentValue.Opposed));
*/

        rpmMap.put(49.0, 3000.0); 
        rpmMap.put(140.0, 5000.0);
  }

  // METHODS START HERE:

  public void MainPID(double targetMainVelocity) {//the target velocity must be in revs per second
    leftMainMotor.setControl(leftMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.ShooterConstants.KV_MAIN));
    rightMainMotor.setControl(rightMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.ShooterConstants.KV_MAIN));
  }

  public void spinMainMotor (double manualMainSpeed) {
    leftMainMotor.set(manualMainSpeed); // between -1 and 1
    rightMainMotor.set(manualMainSpeed); // between -1 and 1
  }

  // this is motor speed between -1.0 and 1.0
  public double getLeftMainSpeed() {
    return leftMainMotor.get();
  }

  public double getRightMainSpeed() {
    return rightMainMotor.get();
  }

  public double getLeftMainVelocity() { //Returns velocity in RPM
    return 60 * leftMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getRightMainVelocity() {
    return 60 * rightMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double calculateRPM(double distance) {
    return rpmMap.get(distance);
  }

  public void stopMain(){
      leftMainMotor.stopMotor();
      rightMainMotor.stopMotor();
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left main velocity: ", getLeftMainVelocity());
    SmartDashboard.putNumber("right main velocity: ", getRightMainVelocity());
  }

}