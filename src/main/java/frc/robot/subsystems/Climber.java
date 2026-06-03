package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX climbMotor;
  private TalonFXConfiguration motorConfig;
  private MotionMagicVoltage m_request;
  private PositionVoltage m_positionRequest;
  private boolean isTopException = false;
  private boolean isBottomException = false;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  private Servo lock;

  public Climber() {
    lock = new Servo(Constants.ClimberConstants.PWM_CLIMB_LOCK);
    lock.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

    climbMotor = new TalonFX(Constants.MotorControllers.ID_CLIMBER, "rio");

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0Configs = motorConfig.Slot0;

    slot0Configs.kS = 0.25;
    slot0Configs.kV = 0.12;
    slot0Configs.kA = 0.01;
    slot0Configs.kP = 2;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    Slot1Configs slot1Configs = motorConfig.Slot1;

    slot1Configs.kP = Constants.ClimberConstants.CLIMBER_KP;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;

    MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 120;
    motionMagicConfigs.MotionMagicJerk = 1200;

    climbMotor.getConfigurator().apply(motorConfig);

    m_request = new MotionMagicVoltage(0).withSlot(0);
    m_positionRequest = new PositionVoltage(0).withSlot(1);

    try {
      topLimitSwitch = new DigitalInput(Constants.ClimberConstants.DIO_CLIMBER_TOP);
    } catch (Exception e) {
      isTopException = true;
    }

    try {
      bottomLimitSwitch = new DigitalInput(Constants.ClimberConstants.DIO_CLIMBER_BOTTOM);
    } catch (Exception e) {
      isBottomException = true;
    }
  }

  public boolean isTopLimit() {
    if (isTopException) {
      return true;
    }
    return topLimitSwitch.get() || getClimberEncoder() > Constants.ClimberConstants.MAX_ENCODER_REVS;
  }

  public boolean isBottomLimit() {
    if (isBottomException) {
      return true;
    }
    return bottomLimitSwitch.get();
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

  public void PIDControlToPosition(double desiredRevs) {
    if (isBottomLimit() && (desiredRevs < getClimberEncoder())) {
      resetClimberEncoder();
      stopClimber();
    } else if (isTopLimit() && (desiredRevs > getClimberEncoder())) {
      stopClimber();
    } else {
      climbMotor.setControl(m_positionRequest.withPosition(desiredRevs));
    }
  }

  public void setLock(double desiredPosition) {
    lock.set(desiredPosition);
  }

  public void retractLock() {
    lock.set(0.0);
  }

  public double getLockPosition() {
    return lock.get();
  }

  public void latchLock() {
    lock.setZeroLatch();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber encoder revs", getClimberEncoder());
    SmartDashboard.putNumber("Climber motor speed", getClimberSpeed());
    SmartDashboard.putBoolean("Climber top limit EXCEPTION", isTopException);
    SmartDashboard.putBoolean("Climber bottom limit EXCEPTION", isBottomException);
    SmartDashboard.putBoolean("Climber top limit hit", isTopLimit());
    SmartDashboard.putBoolean("Climber bottom limit hit", isBottomLimit());
    SmartDashboard.putNumber("Lock Position: ", getLockPosition());
  }

}
