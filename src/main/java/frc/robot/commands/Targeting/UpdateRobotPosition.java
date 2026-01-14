// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateRobotPosition extends InstantCommand {
  private Swerve s_Swerve;

  public UpdateRobotPosition(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      // getting the robots initial pose
      Pose2d robotFieldPose = new Pose2d();
      if (alliance.get() == Alliance.Red) {
        robotFieldPose = LimelightHelpers.getBotPose2d_wpiRed("limelight");
        s_Swerve.setPose(robotFieldPose);

        // double x1 = robotFieldPose.getX();
        // double y1 = robotFieldPose.getY();
        // double angle1 = robotFieldPose.getRotation().getRadians();

        /*
         * SmartDashboard.putNumber("Robot Field X (m):", x1);
         * SmartDashboard.putNumber("x1: ", Units.metersToInches(x1));
         * SmartDashboard.putNumber("Robot Field Y (m):", y1);
         * SmartDashboard.putNumber("y1: ", Units.metersToInches(y1));
         * SmartDashboard.putNumber("angle1",
         * robotFieldPose.getRotation().getDegrees());
         */
        // Field2d field = new Field2d();
        // SmartDashboard.putData(field);
        // field.setRobotPose(x1, y1, new Rotation2d(angle1));
      } else if (alliance.get() == Alliance.Blue) {
        robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        robotFieldPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;
        s_Swerve.setPose(robotFieldPose);

        // double x1 = robotFieldPose.getTranslation().getX();
        // double y1 = robotFieldPose.getTranslation().getY();
        // double angle1 = robotFieldPose.getRotation().getRadians();

        /*
         * SmartDashboard.putNumber("Robot Field X (m):", x1);
         * SmartDashboard.putNumber("x1: ", Units.metersToInches(x1));
         * SmartDashboard.putNumber("Robot Field Y (m):", y1);
         * SmartDashboard.putNumber("y1: ", Units.metersToInches(y1));
         * SmartDashboard.putNumber("angle1",
         * robotFieldPose.getRotation().getDegrees());
         */
        // Field2d field = new Field2d();
        // SmartDashboard.putData(field);
        // field.setRobotPose(x1, y1, new Rotation2d(angle1));
      }
    }
  }
}