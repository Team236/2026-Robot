package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MainRoller extends SubsystemBase {

  private TalonFX leftMainMotor, rightMainMotor;

  private TalonFXConfiguration leftMainConfig, rightMainConfig;

  private VelocityVoltage leftMain_m_request, rightMain_m_request;

  public boolean useInitialBoost = false;

  public MainRoller() {

    leftMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_LEFT_MAIN, "rio");
    leftMainConfig = new TalonFXConfiguration();
    leftMainConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    leftMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    var slot0LMConfigs = leftMainConfig.Slot0;
    slot0LMConfigs.kV = Constants.ShooterConstants.KV_MAIN;
    slot0LMConfigs.kP = Constants.ShooterConstants.KP_MAIN;
    slot0LMConfigs.kI = Constants.ShooterConstants.KI_MAIN;
    slot0LMConfigs.kD = Constants.ShooterConstants.KD_MAIN;

    leftMainMotor.getConfigurator().apply(leftMainConfig);
    leftMain_m_request = new VelocityVoltage(0).withSlot(0);

    rightMainMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_RIGHT_MAIN, "rio");
    rightMainConfig = new TalonFXConfiguration();
    rightMainConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightMainConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    rightMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    var slot0RMConfigs = rightMainConfig.Slot0;
    slot0RMConfigs.kV = Constants.ShooterConstants.KV_MAIN;
    slot0RMConfigs.kP = Constants.ShooterConstants.KP_MAIN;
    slot0RMConfigs.kI = Constants.ShooterConstants.KI_MAIN;
    slot0RMConfigs.kD = Constants.ShooterConstants.KD_MAIN;

    rightMainMotor.getConfigurator().apply(rightMainConfig);
    rightMain_m_request = new VelocityVoltage(0).withSlot(0);

  }

  public void MainPID(double targetMainVelocity) {
    leftMainMotor.setControl(
        leftMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.ShooterConstants.KV_MAIN));
    rightMainMotor.setControl(
        rightMain_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.ShooterConstants.KV_MAIN));
  }

  public void LeftMainPID(double targetLeftMainVelocity) {
    leftMainMotor.setControl(
        leftMain_m_request.withVelocity(targetLeftMainVelocity).withFeedForward(Constants.ShooterConstants.KV_MAIN));
  }

  public void RightMainPID(double targetRightMainVelocity) {
    rightMainMotor.setControl(
        rightMain_m_request.withVelocity(targetRightMainVelocity).withFeedForward(Constants.ShooterConstants.KV_MAIN));
  }

  public void spinMainMotor(double manualMainSpeed) {
    leftMainMotor.set(manualMainSpeed);
    rightMainMotor.set(manualMainSpeed);
  }

  public void spinLeftMainMotor(double manualLeftMainSpeed) {
    leftMainMotor.set(manualLeftMainSpeed);
  }

  public void spinRightMainMotor(double manualRightMainSpeed) {
    rightMainMotor.set(manualRightMainSpeed);
  }

  public double getLeftMainSpeed() {
    return leftMainMotor.get();
  }

  public double getRightMainSpeed() {
    return rightMainMotor.get();
  }

  public double getLeftMainVelocity() {
    return 60 * leftMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getRightMainVelocity() {
    return 60 * rightMainMotor.getRotorVelocity().getValueAsDouble();
  }

  public double calculateRPM(double distance) {
    return Constants.Targeting.rpmMap.get(distance);
  }

  public double calculateNuetralRPM(double distance) {
    return Constants.Targeting.nuetralMap.get(distance);
  }

  public void stopMain() {
    leftMainMotor.stopMotor();
    rightMainMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left main velocity: ", getLeftMainVelocity());
    SmartDashboard.putNumber("right main velocity: ", getRightMainVelocity());
  }

}