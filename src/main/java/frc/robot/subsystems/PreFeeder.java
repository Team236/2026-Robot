package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PreFeederConstants;

public class PreFeeder extends SubsystemBase {

  private TalonFX preFeederMotor;
  private TalonFXConfiguration motorConfig;
  private VelocityVoltage m_request;

  public static Counter counter;
  public boolean isCounterUnplugged = false;
  public boolean isSensorUnplugged = false;
  public DigitalInput lightSensorState;

  public PreFeeder() {
    preFeederMotor = new TalonFX(Constants.MotorControllers.ID_PRE_FEEDER, "rio");

    motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0Configs = motorConfig.Slot0;
    slot0Configs.kV = Constants.PreFeederConstants.KV_PF;
    slot0Configs.kP = PreFeederConstants.KP_PF;
    slot0Configs.kI = PreFeederConstants.KI_PF;
    slot0Configs.kD = PreFeederConstants.KD_PF;

    preFeederMotor.getConfigurator().apply(motorConfig);

    m_request = new VelocityVoltage(0).withSlot(0);

    try {
      lightSensorState = new DigitalInput(Constants.PreFeederConstants.DIO_COUNTER);
    } catch (Exception e) {
      isSensorUnplugged = true;
      SmartDashboard.putBoolean("is lightSensor unplugged:", isSensorUnplugged);
    }

    try {
      counter = new Counter();
      counter.setUpSource(Constants.PreFeederConstants.DIO_COUNTER);
      counter.reset();
    } catch (Exception e) {
      isCounterUnplugged = true;
      SmartDashboard.putBoolean("is lightSensor unplugged:", isSensorUnplugged);
    }

  }

  public int getCount() {
    int count;
    if (isCounterUnplugged) {
      count = 0;
      SmartDashboard.putBoolean("Intake counter unplugged:", isCounterUnplugged);
    } else {
      count = counter.get();
    }
    return count;
  }

  public boolean getLightSensorState() {
    boolean sensorState;
    if (isSensorUnplugged) {
      sensorState = false;
      SmartDashboard.putBoolean("LightSensor is unplugged", isCounterUnplugged);
    } else {
      sensorState = lightSensorState.get();
    }
    return sensorState;
  }

  public void resetCount() {
    counter.reset();
  }

  public double getPreFeederSpeed() {
    return preFeederMotor.get();
  }

  public void setPreFeederSpeed(double speed) {
    preFeederMotor.set(speed);
  }

  public void PreFeederPID(double targetTopVelocity) {
    preFeederMotor
        .setControl(m_request.withVelocity(targetTopVelocity).withFeedForward(Constants.PreFeederConstants.KV_PF));
  }

  public double getPreFeederVelocity() {
    return 60 * preFeederMotor.getRotorVelocity().getValueAsDouble();
  }

  public double calculateRPM(double distance) {
    return Constants.Targeting.preFeederMap.get(distance);
  }

  public void stopPreFeeder() {
    preFeederMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("is counter unplugged:", isCounterUnplugged);
    SmartDashboard.putBoolean("is sensor unplugged:", isSensorUnplugged);
    SmartDashboard.putBoolean("PreFeeder Sensor State is:", getLightSensorState());
    SmartDashboard.putNumber("PreFeeder speed:", getPreFeederSpeed());
    SmartDashboard.putNumber("PreFeeder velocity: ", getPreFeederVelocity());
  }
}
