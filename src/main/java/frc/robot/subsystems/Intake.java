package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public Intake() {

    intakeMotor = new TalonFX(Constants.MotorControllers.ID_INTAKE_LEFT, "rio");
    talonConfig = new TalonFXConfiguration();

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

  }

  public void IntakePID(double targetMainVelocity) {
    double motorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

    double motorRPS = intakeMotor.getVelocity().getValueAsDouble();
    double motorRPM = motorRPS * 60.0;

    double STALL_CURRENT = 18.0;
    double STALL_RPM = 1500.0;

    if (motorCurrent >= STALL_CURRENT && Math.abs(motorRPM) < STALL_RPM) {
      RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    } else {
      RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    intakeMotor
        .setControl(intake_m_request.withVelocity(targetMainVelocity).withFeedForward(Constants.IntakeConstants.KV_I));
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

  public double getIntakeCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake speed", getIntakeSpeed());

  }
}
