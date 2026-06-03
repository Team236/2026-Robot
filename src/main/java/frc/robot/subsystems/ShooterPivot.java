package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private TalonFX shooterPivotMotor;
  private TalonFXConfiguration motorConfig;
  private PositionVoltage m_positionRequest;;
  private boolean isShooterPivotExtException = false;
  private boolean isShooterPivotRetException = false;
  private DigitalInput shooterExtLimit;
  private DigitalInput shooterRetLimit;

  private PositionVoltage m_request;

  public ShooterPivot() {

    shooterPivotMotor = new TalonFX(Constants.MotorControllers.ID_SHOOTER_PIVOT, "rio");
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0Configs = motorConfig.Slot0;
    slot0Configs.kP = Constants.ShooterPivotConstants.KP;
    slot0Configs.kI = Constants.ShooterPivotConstants.KI;
    slot0Configs.kD = Constants.ShooterPivotConstants.KD;

    shooterPivotMotor.getConfigurator().apply(motorConfig);

    m_positionRequest = new PositionVoltage(0).withSlot(0);

    try {
      shooterExtLimit = new DigitalInput(Constants.ShooterPivotConstants.DIO_EXT_LIMIT);
    } catch (Exception e) {
      isShooterPivotExtException = true;
      SmartDashboard.putBoolean(
          "exception thrown for Shooter Extend limit: ",
          isShooterPivotExtException);
    }

    try {
      shooterRetLimit = new DigitalInput(Constants.ShooterPivotConstants.DIO_RET_LIMIT);
    } catch (Exception e) {
      isShooterPivotRetException = true;
      SmartDashboard.putBoolean(
          "exception thrown for Shooter Retract limit: ",
          isShooterPivotRetException);
    }
  }

  public double getEncoderRevs() {
    return shooterPivotMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder() {
    shooterPivotMotor.setPosition(0);
  }

  public void stopShooterPivot() {
    shooterPivotMotor.stopMotor();
  }

  public double getShooterPivotVelocity() {
    return shooterPivotMotor.getVelocity().getValueAsDouble();
  }

  public double getMotorSpeed() {
    return shooterPivotMotor.get();
  }

  public boolean isShooterExtLimit() {
    return shooterExtLimit.get();
  }

  public boolean isShooterRetLimit() {
    return shooterRetLimit.get();
  }

  public boolean isFullyExtended() {
    return (shooterExtLimit.get() || getEncoderRevs() >= Constants.ShooterPivotConstants.ENC_REVS_MAX);
  }

  public boolean isFullyRetracted() {
    return shooterRetLimit.get();
  }

  private void manualSetSpeed(double speed) {
    shooterPivotMotor.set(speed);
  }

  public void manualSetSpeedSafe(double speed) {
    if (isShooterRetLimit() && speed <= 0) {
      resetEncoder();
      stopShooterPivot();
    } else if ((isFullyExtended()) && speed > 0) {
      stopShooterPivot();
    } else {
      manualSetSpeed(speed);
    }
  }

  public double calculateHoodAngle(double distance) {
    return Constants.Targeting.hoodAngleMap.get(distance);
  }

  public void pidSetPosition(double targetRevs) {

    targetRevs = Math.max(0.0, Math.min(targetRevs, Constants.ShooterPivotConstants.ENC_REVS_MAX));

    if (targetRevs > getEncoderRevs() && isFullyExtended()) {
      stopShooterPivot();
      return;
    }

    if (targetRevs < getEncoderRevs() && (isShooterRetLimit() || isFullyRetracted())) {
      stopShooterPivot();
      resetEncoder();
      return;
    }
    shooterPivotMotor.setControl(m_positionRequest.withPosition(targetRevs));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
        "Shooter Pivot Extend Limit?",
        isShooterExtLimit());
    SmartDashboard.putBoolean(
        "Shooter Pivot Retract Limit?",
        isShooterRetLimit());
    SmartDashboard.putNumber(
        "# Shooter Pivot Encoder Revolutions",
        getEncoderRevs());
    SmartDashboard.putBoolean(
        "Shooter Pivot is fully extended?",
        isFullyExtended());
  }
}
