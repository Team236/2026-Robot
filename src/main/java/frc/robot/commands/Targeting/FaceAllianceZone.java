// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FaceAllianceZone extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private BooleanSupplier robotCentricSup;
  private double rotationInput;

  public FaceAllianceZone(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
    // USING SWERVE FOR TAKING OVER ROTATION
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    // var alliance = DriverStation.getAlliance();

    // // ALIGENCE SELECTION FROM DRIVER STATION
    // if (alliance.isPresent() && alliance.get() == Alliance.Red) {

    // } else {

    // }
  }

  @Override
  public void execute() {
    // X AND Y VALUES FOR DRIVING
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

    rotationInput = this.s_Swerve.calculateFaceAlliancePID();

    // DRIVING COMMAND THAT JUST INPUTS COMPUTERS ROTATION
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        MathUtil.clamp(rotationInput, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity),
        !robotCentricSup.getAsBoolean(),
        true
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
