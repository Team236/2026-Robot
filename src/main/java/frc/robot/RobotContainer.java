// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.FuelShooting.PIDShootAll;
import frc.robot.commands.FuelShooting.ShooterMotorManual;
import frc.robot.commands.FuelShooting.SpinLeftMainPID;
import frc.robot.commands.FuelShooting.SpinLeftTopPID;
import frc.robot.commands.FuelShooting.ShootPID;
import frc.robot.commands.FuelShooting.SpinRightMainPID;
import frc.robot.commands.FuelShooting.SpinRightTopPID;
import frc.robot.commands.ClimberCommands.ClimberLock;
import frc.robot.commands.ClimberCommands.ClimberMotionMagic;
import frc.robot.commands.ClimberCommands.ClimberSetSpeed;
import frc.robot.commands.Floor.RunFloor;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunOuttake;
// import frc.robot.commands.CoralHoldCommands.CoralSeqGrabCount;
import frc.robot.commands.PathPlanner.SequentialPathTest;
import frc.robot.commands.PathPlanner.SequentialPathTest2;
import frc.robot.commands.PathPlanner.SequentialPathTest3;
import frc.robot.commands.PathPlanner.SequentialPathsCombined;
import frc.robot.subsystems.ShootMainRoller;
import frc.robot.commands.ShooterPivotCommands.ManualPivot;
import frc.robot.commands.ShooterPivotCommands.PIDPivot;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.BinRelease.ManualMove;
import frc.robot.commands.BinRelease.PIDMove;
import frc.robot.subsystems.BinRelease;


public class RobotContainer {
  
  // controllers
  XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

  // auto switches
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);

  // drive controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // drive buttons
  private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

  // subsystems
  private final ShooterPivot shooterPivot = new ShooterPivot();
  private final ShootMainRoller  fuelShooter = new ShootMainRoller();
  private final BinRelease binRelease = new BinRelease();
  private final Swerve s_Swerve = new Swerve();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();  
  private final Floor floor = new Floor();

  // commands
  private final ManualPivot manualPivotExtend = new ManualPivot(shooterPivot, Constants.Pivot.CONSTANT_FORWARD_SPEED);
  private final ManualPivot manualPivotRetract = new ManualPivot(shooterPivot, Constants.Pivot.CONSTANT_REVERSE_SPEED);
  private final PIDPivot pidPivot = new PIDPivot(shooterPivot, Constants.Pivot.TARGET_REVS);
  private final ManualMove manualExtend = new ManualMove(binRelease, Constants.Bin.MANUAL_EXT_SPEED); // TBD TESTING VALUES
  private final ManualMove manualRetract = new ManualMove(binRelease, Constants.Bin.MANUAL_RET_SPEED); // TBD TESTING VALUES
  private final PIDMove pidToPositionTestA = new PIDMove(binRelease, Constants.Bin.POSITION1); // TBD TESTING VALUES, PID VALUES NEEDED
  private final ClimberMotionMagic climberMotionMagicTest = new ClimberMotionMagic(climber, Constants.Climb.TEST_MM_REVS);
  private final ClimberSetSpeed climberManualUp = new ClimberSetSpeed(climber, Constants.Climb.CLIMBER_UP_SPEED);
  private final ClimberSetSpeed climberManualDown = new ClimberSetSpeed(climber, Constants.Climb.CLIMBER_DOWN_SPEED);
  private final ClimberLock climberLock = new ClimberLock(climber, 0.2); // TBD TESTING VALUE

  private final ShooterMotorManual shooterMotorManual = new ShooterMotorManual(fuelShooter, Constants.Shooter.MAIN_MOTOR_SPEED, Constants.Shooter.TOP_MOTOR_SPEED);
  private final SpinLeftMainPID spinLeftMainPID = new SpinLeftMainPID(fuelShooter, Constants.Shooter.MAIN_MOTOR_RPM);
  private final SpinRightMainPID  spinRightMainPID = new SpinRightMainPID(fuelShooter, Constants.Shooter.MAIN_MOTOR_RPM);
  private final ShootPID spinMidMainPID = new ShootPID(fuelShooter, Constants.Shooter.MAIN_MOTOR_RPM);
  private final SpinLeftTopPID spinLeftTopPID = new SpinLeftTopPID(fuelShooter, Constants.Shooter.TOP_MOTOR_RPM);
  private final SpinRightTopPID spinRightTopPID = new SpinRightTopPID(fuelShooter, Constants.Shooter.TOP_MOTOR_RPM);
  private final PIDShootAll pidShootAll = new PIDShootAll(fuelShooter);
    
  private final RunIntake runIntakeTest = new RunIntake(intake, Constants.Intake.INTAKE_SPEED);
  private final RunOuttake runOuttakeTest = new RunOuttake(intake, Constants.Intake.OUTTAKE_SPEED);
  private final RunFloor runFloorTesting = new RunFloor(floor, Constants.FloorC.TEST_SPEED);
  
  // robot container -- contains subsystems, OI devices, and commands
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve, 
        () -> -driverController.getRawAxis(translationAxis), 
        () -> -driverController.getRawAxis(strafeAxis), 
        () -> -driverController.getRawAxis(rotationAxis), 
        () -> robotCentric.getAsBoolean()
      )
    );

    configureBindings();
  }

  private void configureBindings() {

    // driver controller
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
    JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y); // swerve

    JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB); // swerve
    JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB); // bin (testing)
    JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);

    JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);

    POVButton rightPov = new POVButton(driverController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    POVButton leftPov = new POVButton(driverController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton upPov = new POVButton(driverController,Constants.XboxController.POVXbox.UP_ANGLE); // bin (testing)
    POVButton downPov = new POVButton(driverController,Constants.XboxController.POVXbox.DOWN_ANGLE); // bin (testing)

    Trigger lt = new Trigger(() -> driverController.getRawAxis(Constants.XboxController.AxesXbox.LTrig) > 0.5);
    Trigger rt = new Trigger(() -> driverController.getRawAxis(Constants.XboxController.AxesXbox.RTrig) > 0.5);

    // aux controller
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);

    JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);

    JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);

    POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    
    Trigger lt1 = new Trigger(() -> auxController.getRawAxis(Constants.XboxController.AxesXbox.LTrig) > 0.5);
    Trigger rt1 = new Trigger(() -> auxController.getRawAxis(Constants.XboxController.AxesXbox.RTrig) > 0.5);

    // command binds
    // a.onTrue(algaeGrab).onTrue(l3_Score); *EXAMPLE

    // Fuel Shooter
    // a.whileTrue(shooterMotorManual);
    // b.whileTrue(spinMain);
    // y.whileTrue(spinTopPID);
    // x.whileTrue(pidShootAll);

    // Shooter Pivot
     x.onTrue(pidPivot);
     b.whileTrue(manualPivotExtend);
     a.whileTrue(manualPivotRetract);

    // Bin Release
    // upPov.whileTrue(manualExtend);
    // downPov.whileTrue(manualRetract);
    // b.onTrue(pidToPositionTestA);

    // Climber
    // x.onTrue(climberMotionMagicTest);
    // b.whileTrue(climberManualUp);
    // a.whileTrue(climberManualDown);
    // y.whileTrue(climberLock);
    
    // Intake
     // a.whileTrue(runIntakeTest);
    // b.whileTrue(runOuttakeTest);

    // Feeder
    // b.whileTrue(runFloorTesting);
  }

  public Command getAutonomousCommand() {
    return AutoSwitchHelpers.getCommand();
  }
}