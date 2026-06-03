package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.FuelShooting.Eject;
import frc.robot.commands.FuelShooting.EjectWithOuttake;
import frc.robot.commands.ClimberCommands.ClimberPID;
import frc.robot.commands.ClimberCommands.ClimberSetSpeed;
import frc.robot.commands.Intake.IntakeWithBinExtend;
import frc.robot.subsystems.MainRoller;
import frc.robot.commands.ShooterPivotCommands.ManualPivot;
import frc.robot.commands.Targeting.AlignAndFullClimb;
import frc.robot.commands.Targeting.AutoPrepShooter;
import frc.robot.commands.Targeting.PathPlannerTarget;
import frc.robot.commands.Targeting.SmartAutoTarget;
import frc.robot.commands.Targeting.ShakeAutoTarget;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.PreFeeder;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.BinRelease.ManualMove;
import frc.robot.commands.BinRelease.OverrideMove;
import frc.robot.commands.BinRelease.PIDMove;
import frc.robot.subsystems.BinRelease;

public class RobotContainer {

    public static final XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
    public static final XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

    private final SendableChooser<Command> autoChooser;
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driverController,
            XboxController.Button.kLeftBumper.value);

    private final ShooterPivot shooterPivot = new ShooterPivot();
    private final MainRoller mainRoller = new MainRoller();
    private final BinRelease binRelease = new BinRelease();
    private final Swerve s_Swerve = new Swerve();
    private final Climber climber = new Climber();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final PreFeeder preFeeder = new PreFeeder();

    private final ManualPivot manualPivotExtend = new ManualPivot(shooterPivot,
            Constants.ShooterPivotConstants.CONSTANT_FORWARD_SPEED);
    private final ManualPivot manualPivotRetract = new ManualPivot(shooterPivot,
            Constants.ShooterPivotConstants.CONSTANT_REVERSE_SPEED);

    private final ManualMove binManualExtend = new ManualMove(binRelease,
            Constants.BinReleaseConstants.MANUAL_EXT_SPEED);
    private final ManualMove binManualRetract = new ManualMove(binRelease,
            Constants.BinReleaseConstants.MANUAL_RET_SPEED);

    private final ClimberPID climberPrep = new ClimberPID(climber, Constants.ClimberConstants.PREP_CLIMBER_REVS);
    private final ClimberPID climberSidePrep = new ClimberPID(climber, Constants.ClimberConstants.CLIMB_L1_SIDE_REVS);
    private final ClimberPID climberL1Front = new ClimberPID(climber, Constants.ClimberConstants.CLIMB_L1_FRONT);
    private final ClimberSetSpeed climberManualUp = new ClimberSetSpeed(climber,
            Constants.ClimberConstants.CLIMBER_UP_SPEED);
    private final ClimberSetSpeed climberManualDown = new ClimberSetSpeed(climber,
            Constants.ClimberConstants.CLIMBER_DOWN_SPEED);

    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverController.getRawAxis(translationAxis),
                        () -> -driverController.getRawAxis(strafeAxis),
                        () -> -driverController.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        NamedCommands.registerCommand("shoot",
                new AutoPrepShooter(shooterPivot, mainRoller, s_Swerve, preFeeder, floor, intake, binRelease));
        NamedCommands.registerCommand("prep-climber", climberPrep);
        NamedCommands.registerCommand("climb-l1-front", climberL1Front);
        NamedCommands.registerCommand("prep-side-climber", climberSidePrep);
        NamedCommands.registerCommand("startup-prep", new Eject(preFeeder, mainRoller, floor));
        NamedCommands.registerCommand("intake", new IntakeWithBinExtend(binRelease,
                Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake, Constants.IntakeConstants.INTAKE_RPM));
        NamedCommands.registerCommand("bin-out",
                new PIDMove(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION));
        NamedCommands.registerCommand("bin-zero", new PIDMove(binRelease, 0.0));
        NamedCommands.registerCommand("heading-override", s_Swerve.getPPOverrideHeadingCommand());
        NamedCommands.registerCommand("target", s_Swerve.getPPTargetingCommand());
        NamedCommands.registerCommand("target-with-tolerance", new PathPlannerTarget(s_Swerve));

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> stream.filter(auto -> auto.getName().startsWith("COMP")));
        SmartDashboard.putData("Auto Routine", autoChooser);

        configureBindings();
        configAutos();

    }

    private void configureBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
        JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
        JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
        JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y);

        JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB);
        JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB);
        JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
        JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);

        JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
        JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);

        POVButton rightPov = new POVButton(driverController, Constants.XboxController.POVXbox.RIGHT_ANGLE);
        POVButton leftPov = new POVButton(driverController, Constants.XboxController.POVXbox.LEFT_ANGLE);
        POVButton upPov = new POVButton(driverController, Constants.XboxController.POVXbox.UP_ANGLE);
        POVButton downPov = new POVButton(driverController, Constants.XboxController.POVXbox.DOWN_ANGLE);

        Trigger lt = new Trigger(() -> driverController.getRawAxis(Constants.XboxController.AxesXbox.LTrig) > 0.5);
        Trigger rt = new Trigger(() -> driverController.getRawAxis(Constants.XboxController.AxesXbox.RTrig) > 0.5);

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

        POVButton rightPov1 = new POVButton(auxController, Constants.XboxController.POVXbox.RIGHT_ANGLE);
        POVButton leftPov1 = new POVButton(auxController, Constants.XboxController.POVXbox.LEFT_ANGLE);
        POVButton upPov1 = new POVButton(auxController, Constants.XboxController.POVXbox.UP_ANGLE);
        POVButton downPov1 = new POVButton(auxController, Constants.XboxController.POVXbox.DOWN_ANGLE);

        Trigger lt1 = new Trigger(() -> auxController.getRawAxis(Constants.XboxController.AxesXbox.LTrig) > 0.5);
        Trigger rt1 = new Trigger(() -> auxController.getRawAxis(Constants.XboxController.AxesXbox.RTrig) > 0.5);

        Trigger joystickUp1 = new Trigger(() -> auxController.getRawAxis(XboxController.Axis.kLeftY.value) < -.5);
        Trigger joystickDown1 = new Trigger(() -> auxController.getRawAxis(XboxController.Axis.kLeftY.value) > .5);

        rt.whileTrue(new ShakeAutoTarget(
                s_Swerve,
                () -> -driverController.getRawAxis(translationAxis),
                () -> -driverController.getRawAxis(strafeAxis),
                () -> robotCentric.getAsBoolean()));

        lt.whileTrue(new SmartAutoTarget(
                s_Swerve,
                () -> -driverController.getRawAxis(translationAxis),
                () -> -driverController.getRawAxis(strafeAxis),
                () -> robotCentric.getAsBoolean()));

        lt.onFalse(new InstantCommand(() -> {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }));

        rt.onFalse(new InstantCommand(() -> {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }));

        lm.whileTrue(new AutoPrepShooter(shooterPivot, mainRoller, s_Swerve, preFeeder, floor, intake, binRelease));

        upPov.whileTrue(binManualRetract);
        downPov.whileTrue(binManualExtend);

        b.whileTrue(new AlignAndFullClimb(s_Swerve, binManualExtend, climber, binRelease));

        menu.whileTrue(new EjectWithOuttake(preFeeder, mainRoller, floor, binRelease, intake));

        a.whileTrue(new IntakeWithBinExtend(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake,
                Constants.IntakeConstants.INTAKE_RPM));

        view1.onTrue(climberPrep);
        menu1.onTrue(climberL1Front);
        lm1.onTrue(climberSidePrep);

        upPov1.whileTrue(climberManualUp);
        downPov1.whileTrue(climberManualDown);
        leftPov1.whileTrue(binManualRetract);
        rightPov1.whileTrue(binManualExtend);

        lt1.whileTrue(new OverrideMove(binRelease, 0.25));
        lb1.whileTrue(new OverrideMove(binRelease, -0.25));

        a1.whileTrue(new IntakeWithBinExtend(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION, intake,
                Constants.IntakeConstants.INTAKE_RPM));
        b1.whileTrue(new EjectWithOuttake(preFeeder, mainRoller, floor, binRelease, intake));

        x1.onTrue(new PIDMove(binRelease, 0));
        y1.onTrue(new PIDMove(binRelease, Constants.BinReleaseConstants.BIN_DOWN_POSSITION));

        joystickUp1.whileTrue(manualPivotExtend);
        joystickDown1.whileTrue(manualPivotRetract);

        rt1.whileTrue(new AutoPrepShooter(shooterPivot, mainRoller, s_Swerve, preFeeder, floor, intake, binRelease));
        rb1.whileTrue(new Eject(preFeeder, mainRoller, floor));

    }

    private void configAutos() {

        autoChooser.addOption("COMP_4L_double_nz_double_shoot",
                new PathPlannerAuto("COMP_4R_double_nz_double_shoot", true));
        autoChooser.addOption("COMP_4L_double_nz_single_shoot",
                new PathPlannerAuto("COMP_4R_double_nz_single_shoot", true));
        autoChooser.addOption("COMP_4L_double_nz_double_shoot_greedy",
                new PathPlannerAuto("COMP_4R_double_nz_double_shoot_greedy", true));
        autoChooser.addOption("COMP_4L_double_nz_double_shoot_greedy",
                new PathPlannerAuto("COMP_4R_double_nz_double_shoot_greedy", true));

        autoChooser.addOption("COMP_mirrored-double-trench-shoot",
                new PathPlannerAuto("COMP_double-trench-shoot", true));
        autoChooser.addOption("COMP_3L_bump_bump", new PathPlannerAuto("COMP_3R_bump_bump", true));
        autoChooser.addOption("COMP_3L_bump_trench", new PathPlannerAuto("COMP_3R_bump_trench", true));
        autoChooser.addOption("COMP_3L_bump_bump_GREEDY", new PathPlannerAuto("COMP_3R_bump_bump_GREEDY", true));
        autoChooser.addOption("COMP_3L_bump_bump_no_mid", new PathPlannerAuto("COMP_3R_bump_bump_no_mid", true));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

}