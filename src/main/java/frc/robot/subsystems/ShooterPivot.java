// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//Some of this code is 2024/2025 code, and may not be relevant


public class shooterPivot extends SubsystemBase {
  
  private SparkMax shooterPivotMotor;
  private SparkBaseConfig shooterPivotConfig;
  private Encoder shooterPivotEncoder;
  private boolean isShooterPivotExtException, isShooterPivotRetException;
  private DigitalInput ShooterExtLimit, ShooterRetLimit;
 

    /** Creates a new ShooterPivot. */
    public shooterPivot() {
    shooterPivotMotor = new SparkMax(Constants.MotorControllers.ID_SHOOTER_PIVOT, MotorType.kBrushless);
    shooterPivotEncoder = new Encoder(Constants.ShooterPivot.DIO_ENC_A, Constants.ShooterPivot.DIO_ENC_B);

    shooterPivotConfig.inverted(false);//TODO check if needs to be inverted
    shooterPivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    shooterPivotMotor.configure(shooterPivotConfig,SparkBase.ResetMode.kResetSafeParameters ,SparkBase.PersistMode.kPersistParameters);
        
    try {
      //  This tries to make a new digital input, and if it fails, throws an error 
      ShooterExtLimit = new DigitalInput(Constants.ShooterPivot.DIO_EXT_LIMIT);
    } catch (Exception e) {
       isShooterPivotExtException = true;
      SmartDashboard.putBoolean("exception thrown for Shooter Extend limit: ", isShooterPivotExtException);
    }
    try {
      //  This sets a bottom limit for the shooter, and if it fails, throws an error
      ShooterRetLimit = new DigitalInput(Constants.ShooterPivot.DIO_RET_LIMIT);
    } catch (Exception e) {
      isShooterPivotRetException = true;
      SmartDashboard.putBoolean("exception thrown for Shooter Retract limit: ", isShooterPivotRetException);
    }
}

// methods start here
public double getShooterEncoder() {  //gives encoder reading in Revs
  return shooterPivotEncoder.getRaw();
}

public void resetShooterEncoder() {
  shooterPivotEncoder.reset();
}

public void stopShooterPivot() {
shooterPivotMotor.set(0);
}

public double getShooterPivotSpeed() {
  return shooterPivotMotor.get();
}
public boolean isShooterExtLimit() {
if (isShooterPivotExtException) {
  return true;
} else {
  return ShooterExtLimit.get();
}
}

public boolean isShooterRetLimit() {
if (isShooterPivotRetException) {
  return true;
} else {
  return ShooterRetLimit.get();
}
}

public boolean isFullyExtended() {
  return (getShooterEncoder() <= Constants.ShooterPivot.ENC_REVS_MAX);
}

public void setShooterPivotSpeed(double speed) {
  if (speed > 0) {  //TODO make sure shooter speed > 0 when shooting foward
     //TODO make sure direction is correct
    if (isShooterExtLimit() || isFullyExtended()) {
        // if fully extended limit is tripped or shooter at the maximum desired extension and going out, stop 
        stopShooterPivot();
     }  else {
        //  extending out but fully extended limit is not tripped, go at commanded speed
       shooterPivotMotor.set(speed);
      }
 } 
 else {
      if (isShooterRetLimit()) {
        // shooter retracting and retract limit is tripped, stop and zero encoder
        stopShooterPivot();
        resetShooterEncoder();
      } else {
        // shooter retracting but fully retracted limit is not tripped, go at commanded speed
        ShooterPivotMotor.set(speed);
      }
     }
}



//Begin things that may not be relevant
//these are things that might be useful in the future if we use CANSparkMax PID
//we are not currently using it

//!!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID 
//**** CHANGED BACK TO USING WPILib PID ****
//**** due to spurious encoder polarity changes when run multiple autos in a row ****
/*
public void setSetpoint(double encoderRevs) {
  tiltPIDController.setReference(encoderRevs, ControlType.kPosition);
}

public void setP(double kP) {
  tiltPIDController.setP(kP);
}

public void setI(double kI) {
  tiltPIDController.setI(kI);
}

public void setD(double kD) {
  tiltPIDController.setD(kD);
}

public void setFF(double kFF) {
  tiltPIDController.setFF(kFF);
}
*/


@Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter Pivot Extend Limit?", isShooterExtLimit());
    SmartDashboard.putBoolean("Shooter Pivot Retract Limit?", isShooterRetLimit());
    SmartDashboard.putNumber("# Shooter Pivot Encoder Revolutions", getShooterEncoder());
    SmartDashboard.putBoolean("Shooter Pivot is fully extended?", isFullyExtended());
  }

}