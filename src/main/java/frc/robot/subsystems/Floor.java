package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Floor extends SubsystemBase {

  private TalonFX floorMotor;
  private TalonFXConfiguration motorConfig;
  private VelocityVoltage floor_m_request;

  public Floor() {
    floorMotor = new TalonFX(Constants.MotorControllers.ID_FLOOR, "rio");

    motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0RMConfigs = motorConfig.Slot0;
    slot0RMConfigs.kV = Constants.FloorConstants.KV_F;
    slot0RMConfigs.kP = Constants.FloorConstants.KP_F;
    slot0RMConfigs.kI = Constants.FloorConstants.KI_F;
    slot0RMConfigs.kD = Constants.FloorConstants.KD_F;

    floorMotor.getConfigurator().apply(motorConfig);
    floor_m_request = new VelocityVoltage(0).withSlot(0);
  }

  public void floorPID(double targetMainVelocity) {
    floorMotor
        .setControl(floor_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.FloorConstants.KV_F));
  }

  public double getFloorSpeed() {
    return floorMotor.get();
  }

  public void setFloorSpeed(double speed) {
    floorMotor.set(speed);
  }

  public void stopFloor() {
    floorMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Floor speed", getFloorSpeed());
  }
}
