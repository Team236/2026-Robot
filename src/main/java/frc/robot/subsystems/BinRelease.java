package frc.robot.subsystems;

import java.util.Set;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.BinRelease.PIDMove;

public class BinRelease extends SubsystemBase {

    private TalonFX binReleaseMotor;
    private TalonFXConfiguration motorConfig;
    private DigitalInput maxRetractLimit;
    private boolean isBinRetException;
    private PositionVoltage m_extendRequest;
    private PositionVoltage m_retractRequest;
    private MotionMagicVoltage m_magicRequest;

    private double agitateSetpoint;

    public BinRelease() {
        binReleaseMotor = new TalonFX(Constants.MotorControllers.ID_BIN_REL, "rio");

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var slot0Configs = motorConfig.Slot0;
        slot0Configs.kP = Constants.BinReleaseConstants.KP_BIN_EXTEND;
        slot0Configs.kI = Constants.BinReleaseConstants.KI_BIN_EXTEND;
        slot0Configs.kD = Constants.BinReleaseConstants.KD_BIN_EXTEND;

        var slot1Configs = motorConfig.Slot1;
        slot1Configs.kP = Constants.BinReleaseConstants.KP_BIN_RETRACT;
        slot1Configs.kI = Constants.BinReleaseConstants.KI_BIN_RETRACT;
        slot1Configs.kD = Constants.BinReleaseConstants.KD_BIN_RETRACT;

        Slot2Configs slot2Configs = motorConfig.Slot2;

        slot2Configs.kS = 0;
        slot2Configs.kV = 0;
        slot2Configs.kA = 0;
        slot2Configs.kP = 0;
        slot2Configs.kI = 0;
        slot2Configs.kD = 0;

        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 120;
        motionMagicConfigs.MotionMagicJerk = 1200;

        binReleaseMotor.getConfigurator().apply(motorConfig);
        m_extendRequest = new PositionVoltage(0).withSlot(0);
        m_retractRequest = new PositionVoltage(0).withSlot(1);
        m_magicRequest = new MotionMagicVoltage(0).withSlot(2);

        try {
            maxRetractLimit = new DigitalInput(Constants.BinReleaseConstants.DIO_RET_LIMIT);
        } catch (Exception e) {
            isBinRetException = true;
            SmartDashboard.putBoolean("Exception thrown for binReleaseMaxRetractLimit: ", isBinRetException);
        }

    }

    public void resetEncoder() {
        binReleaseMotor.setPosition(0.0);
    }

    public double getEncoderRevolutions() {
        return binReleaseMotor.getPosition().getValueAsDouble();
    }

    public void stopMotor() {
        binReleaseMotor.set(0);

    }

    public double getMotorSpeed() {
        return binReleaseMotor.get();
    }

    public void manualSetSpeed(double speed) {
        binReleaseMotor.set(speed);
    }

    public boolean isFullyRetracted() {

        return maxRetractLimit.get();
    }

    public boolean isFullyExtended() {

        return (getEncoderRevolutions() > Constants.BinReleaseConstants.ENC_REVS_MAX);
    }

    public void manualSetSpeedSafe(double speed) {
        if (isFullyRetracted() && speed < 0) {
            SmartDashboard.putBoolean("Full Retract so reset encoder", isFullyRetracted());
            resetEncoder();
            stopMotor();
        } else if (isFullyExtended() && speed >= 0) {
            stopMotor();
        } else {
            manualSetSpeed(speed);
        }
    }

    public void PIDControlToPosition(double desiredRevs) {
        if (isFullyRetracted() && (desiredRevs < getEncoderRevolutions())) {
            resetEncoder();
            stopMotor();
        } else if ((isFullyExtended() && desiredRevs > getEncoderRevolutions())) {

            stopMotor();
        } else {
            if (desiredRevs > getEncoderRevolutions()) {
                binReleaseMotor.setControl(m_extendRequest.withPosition(desiredRevs));
            } else {
                binReleaseMotor.setControl(m_retractRequest.withPosition(desiredRevs));
            }
        }
    }

    public void doMotionMagic(double desiredRevs) {
        if (binReleaseMotor.get() > 0 && isFullyExtended()) {
            stopMotor();
            return;
        } else if (binReleaseMotor.get() < 0 && isFullyRetracted()) {
            resetEncoder();
            stopMotor();
        } else {
            binReleaseMotor.setControl(m_magicRequest.withPosition(desiredRevs));
        }
    }

    public Command getAgitateCommand() {
        return new PIDMove(this, 32.5 - 8)
                .until(() -> Math.abs(this.getEncoderRevolutions() - (32.5 - 8)) < 0.2)
                .andThen(
                        new PIDMove(this, 32.5)
                                .until(() -> Math.abs(this.getEncoderRevolutions() - 32.5) < 0.2))
                .repeatedly()
                .finallyDo(() -> this.PIDControlToPosition(32.5));
    }

    public Command getRisingAgitateCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> this.agitateSetpoint = this.getEncoderRevolutions() - 5),

                new DeferredCommand(() -> new PIDMove(this, this.agitateSetpoint)
                        .until(() -> Math.abs(this.getEncoderRevolutions() - this.agitateSetpoint) < 0.2),
                        Set.of(this)),

                new InstantCommand(() -> this.agitateSetpoint = this.getEncoderRevolutions() + 2.5),

                new DeferredCommand(() -> new PIDMove(this, this.agitateSetpoint)
                        .until(() -> Math.abs(this.getEncoderRevolutions() - this.agitateSetpoint) < 0.2),
                        Set.of(this)))
                .repeatedly();
    }

    public Command getManualRisingAgitateCommand() {
        return new DeferredCommand(() -> {
            var state = new Object() {
                double baseline = getEncoderRevolutions();
            };
            double finalTarget = 0.0;

            return new SequentialCommandGroup(
                    new RunCommand(
                            () -> this.manualSetSpeedSafe(-Constants.ChangableBinConstants.BIN_RETRACT_RISING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() <= state.baseline
                                    - Constants.ChangableBinConstants.BIN_RISING_TRAVEL_UP_DISTANCE),

                    new InstantCommand(
                            () -> state.baseline -= Constants.ChangableBinConstants.BIN_RISING_NET_CHANGE_DISTANCE),
                    new RunCommand(
                            () -> this.manualSetSpeedSafe(Constants.ChangableBinConstants.BIN_EXTEND_RISING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() >= state.baseline)
                            .andThen(Commands.waitSeconds(.25)))
                    .repeatedly()
                    .until(() -> this.getEncoderRevolutions() <= finalTarget
                            + Constants.ChangableBinConstants.BIN_AGITATE_END_POSITION)
                    .andThen(this.getManualTopAgitateCommand())
                    .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

    public Command getManualAgitateCommand() {
        return new DeferredCommand(() -> {
            double binOutPose = 32.5;
            double finalTarget = 32.5 - Constants.ChangableBinConstants.BIN_BEGINNING_TRAVEL_DISTANCE;

            return new SequentialCommandGroup(
                    new RunCommand(
                            () -> this.manualSetSpeedSafe(-Constants.ChangableBinConstants.BIN_RETRACT_RISING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() <= finalTarget),

                    new RunCommand(
                            () -> this.manualSetSpeedSafe(Constants.ChangableBinConstants.BIN_EXTEND_BEGINNING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() >= binOutPose)
                            .andThen(Commands.waitSeconds(0.67)))
                    .repeatedly()
                    .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

    public Command getManualSmartAgitateCommand() {
        if (Constants.ChangableBinConstants.USE_RISING_AGITATE) {
            return this.getManualRisingAgitateCommand();
        } else {
            return this.getManualAgitateCommand();
        }
    }

    public Command getManualIncreasingAgitateCommand() {
        return new DeferredCommand(() -> {
            var state = new Object() {
                double baseline = Constants.BinReleaseConstants.BIN_AGITATE_DOWN_POSSITION;
            };
            double finalTarget = 0.0;

            return new SequentialCommandGroup(
                    new RunCommand(
                            () -> this.manualSetSpeedSafe(-Constants.ChangableBinConstants.BIN_RETRACT_RISING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() <= state.baseline
                                    - Constants.ChangableBinConstants.BIN_RISING_TRAVEL_UP_DISTANCE),

                    new InstantCommand(
                            () -> state.baseline -= Constants.ChangableBinConstants.BIN_RISING_NET_CHANGE_DISTANCE),
                    new RunCommand(
                            () -> this.manualSetSpeedSafe(Constants.ChangableBinConstants.BIN_EXTEND_RISING_SPEED),
                            this)
                            .until(() -> this
                                    .getEncoderRevolutions() >= Constants.BinReleaseConstants.BIN_AGITATE_DOWN_POSSITION))
                    .repeatedly()
                    .until(() -> this.getEncoderRevolutions() <= finalTarget
                            + Constants.ChangableBinConstants.BIN_AGITATE_END_POSITION)
                    .andThen(this.getManualTopAgitateCommand())
                    .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

    public Command getManualTopAgitateCommand() {
        return new DeferredCommand(() -> {
            double binTopPose = Constants.ChangableBinConstants.BIN_AGITATE_END_POSITION;

            double finalTarget = binTopPose + Constants.ChangableBinConstants.BIN_BEGINNING_TRAVEL_DISTANCE;

            return new SequentialCommandGroup(
                    new RunCommand(
                            () -> this.manualSetSpeedSafe(Constants.ChangableBinConstants.BIN_EXTEND_BEGINNING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() >= finalTarget),

                    new RunCommand(
                            () -> this.manualSetSpeedSafe(-Constants.ChangableBinConstants.BIN_RETRACT_BEGINNING_SPEED),
                            this)
                            .until(() -> this.getEncoderRevolutions() <= binTopPose))
                    .repeatedly()
                    .finallyDo(() -> this.stopMotor());
        }, Set.of(this));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Bin encoder revolutions:", getEncoderRevolutions());
        SmartDashboard.putBoolean("Bin fully extended:", isFullyExtended());
        SmartDashboard.putBoolean("Bin fully retracted:", isFullyRetracted());
        SmartDashboard.putNumber("Bin Tuning/Bin Speed", getMotorSpeed());
    }
}