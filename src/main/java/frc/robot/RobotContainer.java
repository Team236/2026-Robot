
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
// import frc.robot.commands.CoralHoldCommands.CoralSeqGrabCount;
import frc.robot.commands.PathPlanner.SequentialPathTest;
import frc.robot.commands.PathPlanner.SequentialPathTest2;
import frc.robot.commands.PathPlanner.SequentialPathTest3;
import frc.robot.commands.PathPlanner.SequentialPathsCombined;
import frc.robot.subsystems.Swerve;





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
  private final Swerve s_Swerve = new Swerve();

  // commands
  
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
      
    // named commands
    //NamedCommands.registerCommand("coralGrab", coralGrab); **EXAMPLE

    configureBindings();
  }

  private void configureBindings() {
    
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    // driver controller
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
    JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y); // swerve

    JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB); // swerve
    JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);

    JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);

    POVButton rightPov = new POVButton(driverController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    POVButton leftPov = new POVButton(driverController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton upPov = new POVButton(driverController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driverController,Constants.XboxController.POVXbox.DOWN_ANGLE);

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


    //Inputs

  //y button is already assigned to ZeroGyro
  //leftBumper lb button is already assigned to RobotCentric


//DRIVER CONTROLLER

//driving & gyro
//rb robot centric already binded
//y reset gyro already binded
// rb.onTrue(updateRobotPosition);



// a.onTrue(algaeGrab).onTrue(l3_Score);
// b.onTrue(algaeGrab).onTrue(l4_Score);
  }

  public Command getAutonomousCommand() {

  // SmartDashboard.putString("autokey", "Entering getAutoCommand now");
  //SmartDashboard.putString("Asking for auto sequence", "" + !autoSwitch1.get() + !autoSwitch2.get() + !autoSwitch3.get() + !autoSwitch4.get());
  Command command = null;


  //! means switch is on
  // if (!autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && !autoSwitch4.get()) {
  //   command = fullRunRight;
  // } else if (!autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {
  //   command = legs1and2Right;
  // } else if (!autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
  //   command = leg1Right; 
  // } else if (autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && !autoSwitch4.get()) {
  //   command =  fullRunLeft;
  // } else if (autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {
  //   //  command =  lesssgs1and2Left;
  //   command = leg1PracticeFieldTest;
  //  // command = leg1and2Practice;
  // } else if (autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
  //   command = leg1Left;
  // } else if (autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
  //   command = ctrScore1;
  // } 
  // else if (autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && !autoSwitch4.get()) {
  //   command = centerNoAlgae;
  // } 
 return command;
}

}
    /* 
    // SmartDashboard.putString("autokey", "Entering getAutoCommand now");
    Command command = null;
    AutoSwitchHelpers autoSwitchHelpers = new AutoSwitchHelpers();
    // Switch 1 is in the "ON" spot on the old auto box

    if (autoSwitchHelpers.switchesAre(true, true, true, true)){
      command = fullRunRight;
    } else if (autoSwitchHelpers.switchesAre(true, true, false, false)){
      command = leg1Right;
    } else if (autoSwitchHelpers.switchesAre(true, true, true, false)){
      command = legs1and2Right;
    }  else if (autoSwitchHelpers.switchesAre(false, true, true, true)){
        command = fullRunLeft;
    }  else if (autoSwitchHelpers.switchesAre(false, true, false, false)){
        command = leg1Left;
    }  else if (autoSwitchHelpers.switchesAre(false, true, true, false)){
        command = legs1and2Left;
    }  else if (autoSwitchHelpers.switchesAre(false, false, false, false)){
        command = fullRunCenter;
    }
   return command;
   */
 


