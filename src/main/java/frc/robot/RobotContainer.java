// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.FuelShooting.Eject;
import frc.robot.commands.FuelShooting.EjectWithOuttake;
import frc.robot.commands.FuelShooting.ManualMainRoller;
import frc.robot.commands.FuelShooting.ManualShoot;
import frc.robot.commands.FuelShooting.PIDMainRoller;
import frc.robot.commands.FuelShooting.PIDShoot;
import frc.robot.commands.ClimberCommands.ClimberLock;
import frc.robot.commands.ClimberCommands.ClimberMotionMagic;
import frc.robot.commands.ClimberCommands.ClimberPID;
import frc.robot.commands.ClimberCommands.ClimberSetSpeed;
import frc.robot.commands.ClimberCommands.ZeroClimberStartup;
import frc.robot.commands.Floor.RunFloor;
import frc.robot.commands.Intake.IntakeWithBinExtend;
import frc.robot.commands.Intake.PIDIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunOuttake;
import frc.robot.commands.PreFeeder.PIDPreFeederWithCounter;
import frc.robot.commands.PreFeeder.PIDPrefeeder;
import frc.robot.commands.PreFeeder.RunPreFeeder;
import frc.robot.commands.PreFeeder.RunPreFeederWithCounter;
// import frc.robot.commands.CoralHoldCommands.CoralSeqGrabCount;
import frc.robot.commands.PathPlanner.SequentialPathTest;
import frc.robot.commands.PathPlanner.SequentialPathTest2;
import frc.robot.commands.PathPlanner.SequentialPathTest3;
import frc.robot.commands.PathPlanner.SequentialPathsCombined;
import frc.robot.subsystems.MainRoller;
import frc.robot.commands.ShooterPivotCommands.ManualPivot;
import frc.robot.commands.ShooterPivotCommands.PIDPivot;
import frc.robot.commands.ShooterPivotCommands.ZeroPivotStartup;
import frc.robot.commands.Targeting.AimOnMove;
import frc.robot.commands.Targeting.AlignAndFullClimb;
import frc.robot.commands.Targeting.AutoPivotRobotGroupCommand;
import frc.robot.commands.Targeting.AutoPivotTowardHub;
import frc.robot.commands.Targeting.AutoPrepShooter;
import frc.robot.commands.Targeting.SmartAutoTarget;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.PreFeeder;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.BinRelease.ManualMove;
import frc.robot.commands.BinRelease.OverrideMove;
import frc.robot.commands.BinRelease.Agitate;
import frc.robot.commands.BinRelease.PIDMove;
import frc.robot.subsystems.BinRelease;


public class RobotContainer {
  
  // controllers
  public static final XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  public static final XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

  // // auto switches
  // commented out because these are now declared in AutoSwitchHelpers
  // private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
  // private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
  // private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
  // private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);
  private final SendableChooser<Command> autoChooser;
  // drive controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // drive buttons
  private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

  // subsystems
  private final ShooterPivot shooterPivot = new ShooterPivot();
  private final MainRoller  mainRoller = new MainRoller();
  private final BinRelease binRelease = new BinRelease();
  private final Swerve s_Swerve = new Swerve();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();  
  private final Floor floor = new Floor();
  private final PreFeeder preFeeder = new PreFeeder();

//COMMANDS:

//PIVOT (HOOD)
  private final ManualPivot manualPivotExtend = new ManualPivot(shooterPivot, Constants.ShooterPivotConstants.CONSTANT_FORWARD_SPEED);
  private final ManualPivot manualPivotRetract = new ManualPivot(shooterPivot, Constants.ShooterPivotConstants.CONSTANT_REVERSE_SPEED);
  // private final PIDPivot pidPivot = new PIDPivot(shooterPivot, Constants.ShooterPivotConstants.TARGET_REVS);

//BIN RELEASE
  private final ManualMove binManualExtend = new ManualMove(binRelease, Constants.BinReleaseConstants.MANUAL_EXT_SPEED); // 
  private final ManualMove binManualRetract = new ManualMove(binRelease, Constants.BinReleaseConstants.MANUAL_RET_SPEED); // 
 // private final PIDMove pidToPositionTestA = new PIDMove(binRelease, Constants.BinReleaseConstants.POSITION1); // TBD TESTING VALUES, PID VALUES NEEDED

//CLIMBER
  private final ClimberPID climberPrep = new ClimberPID(climber, Constants.ClimberConstants.PREP_CLIMBER_REVS);
  private final ClimberPID climberL1Side = new ClimberPID(climber, Constants.ClimberConstants.CLIMB_L1_SIDE);
  private final ClimberPID climberL1Front = new ClimberPID(climber, Constants.ClimberConstants.CLIMB_L1_FRONT);
  private final ClimberSetSpeed climberManualUp = new ClimberSetSpeed(climber, Constants.ClimberConstants.CLIMBER_UP_SPEED);
  private final ClimberSetSpeed climberManualDown = new ClimberSetSpeed(climber, Constants.ClimberConstants.CLIMBER_DOWN_SPEED);
 // private final ClimberLock climberLock = new ClimberLock(climber, 1.0); // TBD TESTING VALUE
 // private final ClimberLock climberUnlock = new ClimberLock(climber, 0.0); // TBD TESTING VALUE
 // private final ClimberMotionMagic climberMotionMagicTest = new ClimberMotionMagic(climber, Constants.ClimberConstants.TEST_MM_REVS);

//SHOOTER
 // private final ManualMainRoller manualMainRoller = new ManualMainRoller(mainRoller, Constants.ShooterConstants.MAIN_MOTOR_SPEED);
 private final ManualShoot manualShoot = new ManualShoot(mainRoller, preFeeder);
 // private final PIDMainRoller pidMainRoller = new PIDMainRoller(mainRoller, s_Swerve, Constants.ShooterConstants.MAIN_MOTOR_RPM);
 // private final PIDShoot pidShoot = new PIDShoot(mainRoller, s_Swerve, preFeeder, floor, intake, binRelease);

//INTAKE  
  //private final RunIntake runIntakeTest = new RunIntake(intake, Constants.IntakeConstants.INTAKE_SPEED);
  //private final RunOuttake runOuttakeTest = new RunOuttake(intake, Constants.IntakeConstants.OUTTAKE_SPEED);
  //private final RunFloor runFloorTesting = new RunFloor(floor, Constants.FloorConstants.TEST_SPEED);
  //private final IntakeWithBinExtend intakeWithBinExtend = new IntakeWithBinExtend(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake, Constants.IntakeConstants.DESIRED_RPM);

//PREFEEDER
 // private final RunPreFeeder runPreFeederTesting = new RunPreFeeder(preFeeder, Constants.PreFeederConstants.TEST_SPEED);
 // private final RunPreFeederWithCounter runPreFeederWithCounterTesting = new RunPreFeederWithCounter(preFeeder, Constants.PreFeederConstants.TEST_SPEED);
 // private final PIDPrefeeder pidPrefeeder = new PIDPrefeeder(preFeeder, s_Swerve, Constants.PreFeederConstants.DESIRED_RPM);
 // private final PIDPreFeederWithCounter pidPreFeederWithCounter = new PIDPreFeederWithCounter(preFeeder, Constants.PreFeederConstants.DESIRED_RPM);
  
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

    NamedCommands.registerCommand("shoot", new AutoPrepShooter(shooterPivot, mainRoller, s_Swerve, preFeeder, floor, intake, binRelease));
    NamedCommands.registerCommand("prep-climber", climberPrep);
    NamedCommands.registerCommand("climb-l1-front", climberL1Front);
    NamedCommands.registerCommand("startup-prep", new Eject(preFeeder, mainRoller, floor));
    NamedCommands.registerCommand("intake", new IntakeWithBinExtend(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake, 6500));
    NamedCommands.registerCommand("bin-out", new PIDMove(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION));
    NamedCommands.registerCommand("bin-zero", new PIDMove(binRelease, 0.0));
    NamedCommands.registerCommand("heading-override", s_Swerve.getPPOverrideHeadingCommand());
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> stream.filter(auto -> auto.getName().startsWith("COMP"))
    );
    // // autoChooser = null;
    SmartDashboard.putData("Auto Routine", autoChooser);

    configureBindings();
    configAutos();

  }

  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    // driver controller
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A); // feeder
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B); // feeder
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

    Trigger joystickUp1 = new Trigger(() -> auxController.getRawAxis(XboxController.Axis.kLeftY.value) < -.5);
    Trigger joystickDown1 = new Trigger(() -> auxController.getRawAxis(XboxController.Axis.kLeftY.value) > .5);

    // command binds
    // a.onTrue(algaeGrab).onTrue(l3_Score); *EXAMPLE

    // ********* NOT WORKING ***********
    // view.whileTrue(Commands.defer((() -> s_Swerve.getClimbTargetingPath()), Set.of(s_Swerve)));



    // ---------------------
    // FINAL DRIVER BUTTONS
    // ---------------------

    // zero gyro bind above, field centric driving defined above

    rt.whileTrue(new SmartAutoTarget(
        s_Swerve, 
        () -> -driverController.getRawAxis(translationAxis), 
        () -> -driverController.getRawAxis(strafeAxis), 
        () -> robotCentric.getAsBoolean()
    ));

    rt.onFalse( new InstantCommand(() -> {
        RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      })
    ); 

    upPov.whileTrue(binManualRetract);
    downPov.whileTrue(binManualExtend);

    b.whileTrue(Commands.defer((() -> s_Swerve.getClimbTargetingPath()), Set.of(s_Swerve))); // NOT WORKING WANTS ALIGN SIDE
    x.whileTrue(Commands.defer((() -> s_Swerve.getClimbTargetingPath()), Set.of(s_Swerve))); // NOT WORKING WANTS ALIGN FRONT

    menu.whileTrue(new EjectWithOuttake(preFeeder, mainRoller, floor, binRelease, intake));

    // a.whileTrue(new AutoPrepShooter(shooterPivot, mainRoller, s_Swerve, preFeeder, floor, intake, binRelease));

    // a.whileTrue(new  AlignAndFullClimb(s_Swerve, binManualExtend, climber, binRelease) );
    // a.whileTrue(manualShoot);

    a.whileTrue(new IntakeWithBinExtend(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake, 6500));

    // ------------------
    // FINAL AUX BUTTONS
    // ------------------
    
    // view1.onTrue(climberPrep); //RENABLE ONCE FIXED*
    // menu1.onTrue(climberL1Front); //RENABLE ONCE FIXED*

    upPov1.whileTrue(climberManualUp); //RENABLE ONCE FIXED*
    downPov1.whileTrue(climberManualDown); //RENABLE ONCE FIXED*

    // lm1.whileTrue(binManualRetract);
    // rm1.whileTrue(binManualExtend);

    lt1.whileTrue(new OverrideMove(binRelease, 0.25));
    lb1.whileTrue(new OverrideMove(binRelease, -0.25));

    a1.whileTrue(new IntakeWithBinExtend(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake, 6500));
    b1.whileTrue(new EjectWithOuttake(preFeeder, mainRoller, floor, binRelease, intake));

    x1.onTrue(new PIDMove(binRelease, 0));
    y1.onTrue(new PIDMove(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION));

    joystickUp1.whileTrue(manualPivotExtend);
    joystickDown1.whileTrue(manualPivotRetract);

    rt1.whileTrue (new AutoPrepShooter(shooterPivot, mainRoller, s_Swerve, preFeeder, floor, intake, binRelease));
    rb1.whileTrue(new Eject(preFeeder, mainRoller, floor));

    // -----------------------
    // TESTING COMMANDS BELOW
    // -----------------------

    lm1.whileTrue(binRelease.getManualRisingAgitateCommand());
    rm1.whileTrue(binRelease.getManualAgitateCommand());

    // rm1.onFalse(() -> binRelease.stopMotor());
    // lm1.onFalse(() -> binRelease.stopMotor());

    // rt.whileTrue(new AimOnMove(
    //   s_Swerve, 
    //   () -> -driverController.getRawAxis(translationAxis), 
    //   () -> -driverController.getRawAxis(strafeAxis), 
    //   () -> robotCentric.getAsBoolean(), 
    //   mainRoller,
    //   shooterPivot,
    //   preFeeder
    // ));

  }

  private void configAutos() {
    //    //RIGHT means the OUTPOST side of the field
    //    //LEFT means the DEPOT side of the field

    //OOOO (OFF OFF OFF OFF)  Right Trench w/ Bin facing Drivers, Outpost, Shoot, front Right Tower Climb 
    AutoSwitchHelpers.put(false, false, false, false, new PathPlannerAuto("COMP-trench-outpost-climb"));

    // //OOOX (OFF OFF OFF ON)  Right Trench w/ Bin facing away, Neutral Zone, Right Trench, Shoot, front Right Tower Climb 
    AutoSwitchHelpers.put(false, false, false, true, new PathPlannerAuto("COMP-srt0_neutral_trench_shoot_and_climb"));

    // //OOXO (OFF OFF ON OFF)  Left Trench w/ Bin facing away, Neutral Zone, Left Trench, Shoot, front Left Tower Climb 
    AutoSwitchHelpers.put(false, false, true, false, new PathPlannerAuto("COMP-inverse-srt0_neutral_trench_shoot_and_climb"));

    // //OOXX (OFF OFF ON ON)   Center Hub w/ Bin facing Hub, Shoot (no climb, just move back a few inches and shoot 8 Fuel)
    AutoSwitchHelpers.put(false, false, true, true, new PathPlannerAuto("COMP-hub-shoot"));

    // //OXOO (OFF ON OFF OFF)  Center Hub w/ Bin facing Hub, Shoot, Climb Left front Tower
    AutoSwitchHelpers.put(false, true, false, false, new PathPlannerAuto("COMP-hub-shoot-and-left-climb"));

    // //OXOX (OFF ON OFF ON)   Center Hub w/ Bin facing Hub, Shoot, Climb Right front Tower 
    AutoSwitchHelpers.put(false, true, false, true, new PathPlannerAuto("COMP-hub-shoot-and-right-climb"));
    
    // //NOT COMP TESTED YET - TRY ON THU:
    // //OXXO (OFF ON ON OFF)   Right Bump w/ Bin Facing 90 Left, Nuetral Zone, Bump, Shoot, Climb Right front Tower 
    AutoSwitchHelpers.put(false, true, true, false, new PathPlannerAuto("srbc90_bump_neutral_bump_shoot_and_climb"));

    // //OXXX (OFF ON ON ON)    Right Trench w/ Bin Facing Hub, Neutral Zone, Bump, Shoot
    AutoSwitchHelpers.put(false, true, true, true, new PathPlannerAuto("COMP-srt0_neutral_trench_and_shoot"));

    // //XOOO (ON OFF OFF OFF)    Left Trench w/ Bin Facing Hub, Neutral Zone, Bump, Shoot
    AutoSwitchHelpers.put(true, false, false, false, new PathPlannerAuto("COMP-inverse-srt0_neutral_trench_and_shoot"));

    //XOOX (ON OFF OFF ON)    Right Trench w/ Bin Facing Hub, DOUBLE--Neutral Zone, Trench, Shoot
    AutoSwitchHelpers.put(true, false, false, true, new PathPlannerAuto("COMP_srt0_double_neutral_trench_shoot"));
      
    //XOXO (ON OFF ON OFF)    Left Trench w/ Bin Facing Hub, DOUBLE--Neutral Zone, Trench, Shoot
    AutoSwitchHelpers.put(true, false, true, false, new PathPlannerAuto("COMP_slt0_double_neutral_trench_shoot"));
    
    //XOXX (ON OFF ON ON) 
    AutoSwitchHelpers.put(true, false, true, true, new PathPlannerAuto("COMP_speedy_srt0_double_neutral_trench_shoot"));
      
    //XXOO (ON ON OFF OFF) 
    AutoSwitchHelpers.put(true, true, false, false, new PathPlannerAuto("COMP_speedy_slt0_double_neutral_trench_shoot"));

    //XXOX (ON ON OFF ON) 
    AutoSwitchHelpers.put(true, true, false, true, new PathPlannerAuto("srt90_neutral_bump_shoot_trench_bump_and_shoot_speedy"));
      
    //XXXO (ON ON ON OFF) 
    // AutoSwitchHelpers.put(true, true, true, false, new PathPlannerAuto("?", mirror));

    //!!!!DO NOT USE THE ONE BELOW UNLESS REALLY NEEDED - WE DON'T WANT TO GO OVER THE DEPOT BUMPS!!!
    //XXXX (ON ON ON ON)  Center Hub, Depot, Shoot, Climb Left front Tower 
    // AutoSwitchHelpers.put(true, true, true, true, new PathPlannerAuto("COMP-hub-depot-shoot-and-climb", mirror));
  }
  
  public Command getAutonomousCommand() 
  {  
    return autoChooser.getSelected();

    // PathPlannerAuto pathPlannerAuto = AutoSwitchHelpers.getPathPlannerAuto();
    // return pathPlannerAuto;

    // if (!autoSwitch1.get() && !autoSwitch2.get() && !autoSwitch3.get() && !autoSwitch4.get()) {
    //   return
    // } 
    

    // TODO ALSO: SHOULD LOOK AT PATHPLANNER WEBSITE FOR SENDABLE CHOOSERS; AUTO CAN BE SELECTED ON SMART DASHBOARD OR EQUIVALENT

    // this command adding should really be done up top? instead of loaded all down here
    // AutoSwitchHelpers.put(false, false, false, false, new InstantCommand(() -> SmartDashboard.putString("Auto", "ffff")));
    // AutoSwitchHelpers.put(false, true, false, true, new InstantCommand(() -> SmartDashboard.putString("Auto", "ftft")));
    // AutoSwitchHelpers.put(true, false, true, false, new InstantCommand(() -> SmartDashboard.putString("Auto", "tftf")));
    // AutoSwitchHelpers.put(true, true, true, true, new InstantCommand(() -> SmartDashboard.putString("Auto", "tttt")));
    
    // return AutoSwitchHelpers.getAutoCommand();
    
    // we make paths usually on the blue side,

    //below does not work currently (mirror flips y, does not affect x)
    //boolean shouldMirror = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false;
    // return new PathPlannerAuto("srt0_neutral_trench_shoot_and_climb");

    // ------------------------------------------------
    // AUTO PATHS THAT HAVE BEEN TESTED
    //------------------------------------------------
  
    // Trench to nuetral under trench shoot and climb: srt0_neutral_trench_shoot_and_climb (works)
    // Hub center to depot intake, shoot, and climb: hub-depot-shoot-and-climb (intake needs work)
    // Trench to outpost shoot and climb: trench-to-outpost (works)
    // Just backs up and shoots: hub-shoot (works)
    // Just backs up, shoots, and climbs left side: hub-shoot-and-left-climb (works)


  }

}