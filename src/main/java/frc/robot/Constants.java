package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;

/**
 * The master Constants file. This contains numeric values and configurations
 * for the entire robot.
 * Accessing variables here prevents hardcoded "magic numbers" scattered
 * throughout the code,
 * minimizing errors and allowing the team to edit robot values from a single
 * location.
 */

public final class Constants {
  public static final double stickDeadband = 0.1;

  /**
   * Holds the USB port IDs for the driver station controllers.
   * These map directly to the order the controllers are plugged into the driver
   * station laptop.
   */

  public static final class Controller {
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;
  }

  /**
   * Defines the unique CAN bus IDs for every motor controller on the robot.
   * These IDs must match the physical configuration set in the Phoenix Tuner or
   * REV Hardware Client.
   */

  public static class MotorControllers {
    public static final int SMART_CURRENT_LIMIT = 40;
    public static final int INTAKE_CURRENT_LIMIT = 32;
    public static final int ID_BIN_REL = 8;
    public static final int ID_INTAKE_LEFT = 9;
    public static final int ID_FLOOR = 10;
    public static final int ID_SHOOTER_LEFT_MAIN = 12;
    public static final int ID_SHOOTER_RIGHT_MAIN = 11;
    public static final int ID_SHOOTER_PIVOT = 15;
    public static final int ID_CLIMBER = 13;
    public static final int ID_PRE_FEEDER = 14;
  }

  /**
   * Constants for the main shooter mechanism.
   * Includes target RPMs and the PIDF tuning values required to keep the
   * flywheels spinning at a stable speed.
   */

  public static class ShooterConstants {
    public static final double MAIN_MOTOR_RPM = 2550;
    public static final double MAIN_MOTOR_SPEED = 0.3;
    public static final double KV_MAIN = 0.122;
    public static final double KP_MAIN = 0.63;
    public static final double KI_MAIN = 0;
    public static final double KD_MAIN = 0;
  }

  /**
   * Constants for the pre-feeder mechanism.
   * This mechanism stages game pieces and feeds them into the main shooter
   * flywheels at the precise moment.
   */

  public static class PreFeederConstants {
    public static final int DIO_COUNTER = 10;
    public static final double TEST_SPEED = 0.3;
    public static final int DESIRED_RPM = 2550;
    public static final double KV_PF = 0.0983;
    public static final double KP_PF = 0.97;
    public static final double KI_PF = 0;
    public static final double KD_PF = 0;
  }

  /**
   * Constants for the Bin Release mechanism.
   * Contains DIO (Digital Input/Output) ports for limit switches and PID values
   * for extending/retracting the bin.
   */

  public static final class BinReleaseConstants {

    public static final int DIO_EXT_LIMIT = 8;
    public static final int DIO_RET_LIMIT = 9;

    public static final double KP_BIN_EXTEND = 1.0;
    public static final double KI_BIN_EXTEND = 0;
    public static final double KD_BIN_EXTEND = 0;

    public static final double KP_BIN_RETRACT = 1.5;
    public static final double KI_BIN_RETRACT = 0;
    public static final double KD_BIN_RETRACT = 0;

    public static final double ENC_REVS_MAX = 31.8;
    public static final double MANUAL_EXT_SPEED = 0.3;
    public static final double MANUAL_RET_SPEED = -0.3;
    public static final double POSITION1 = 50;
    public static final double AGITATE_POSITION = 7.0;
    public static final double AGITATE_TOLERANCE = 0.5;
    public static final double AGITATE_TEST_SPEED = 0.2;

    public static final double BIN_DOWN_POSSITION = 31.8;

    public static final double BIN_AGITATE_DOWN_POSSITION = 31.0;
  }

  /**
   * Variables in this class are pushed to and pulled from SmartDashboard.
   * This allows the drive team to adjust bin timing and speeds on the fly without
   * redeploying code.
   */

  public static class ChangableBinConstants {

    public static double BIN_WAIT_TIME = 1.25;
    public static double BIN_BEGINNING_TIME = 2.0;
    public static double BIN_BEGINNING_TRAVEL_DISTANCE = 8.0;
    public static double BIN_RETRACT_BEGINNING_SPEED = 0.3;
    public static double BIN_EXTEND_BEGINNING_SPEED = 0.35;
    public static double BIN_RETRACT_RISING_SPEED = 0.35;
    public static double BIN_EXTEND_RISING_SPEED = 0.3;
    public static double BIN_RISING_TRAVEL_UP_DISTANCE = 9.0;
    public static double BIN_RISING_NET_CHANGE_DISTANCE = 4.5;
    public static double BIN_AGITATE_END_POSITION = 7.5;
    public static boolean USE_RISING_AGITATE = true;

    /**
     * Sends the current values of each variable to the dashboard.
     */

    public static void pushToDashboard() {
      SmartDashboard.putNumber("Bin Tuning/Agitate End Position", BIN_AGITATE_END_POSITION);
      SmartDashboard.putNumber("Bin Tuning/Retract Beginning Speed", BIN_RETRACT_BEGINNING_SPEED);
      SmartDashboard.putNumber("Bin Tuning/Extend Beginning Speed", BIN_EXTEND_BEGINNING_SPEED);
      SmartDashboard.putNumber("Bin Tuning/Retract Rising Speed", BIN_RETRACT_RISING_SPEED);
      SmartDashboard.putNumber("Bin Tuning/Extend Rising Speed", BIN_EXTEND_RISING_SPEED);
      SmartDashboard.putNumber("Bin Tuning/Beginning Travel Distance", BIN_BEGINNING_TRAVEL_DISTANCE);
      SmartDashboard.putNumber("Bin Tuning/Rising Travel Up Distance", BIN_RISING_TRAVEL_UP_DISTANCE);
      SmartDashboard.putNumber("Bin Tuning/Rising Net Change Distance", BIN_RISING_NET_CHANGE_DISTANCE);
      SmartDashboard.putNumber("Bin Tuning/Bin Wait Time", BIN_WAIT_TIME);
      SmartDashboard.putNumber("Bin Tuning/Bin Beggining Time", BIN_BEGINNING_TIME);
      SmartDashboard.putBoolean("Bin Tuning/Use Rising", USE_RISING_AGITATE);
    }

    /**
     * Retrieves numbers assigned on the dashboard and updates local variables.
     */

    public static void pullFromDashboard() {
      BIN_AGITATE_END_POSITION = SmartDashboard.getNumber("Bin Tuning/Agitate End Position", BIN_AGITATE_END_POSITION);
      BIN_RETRACT_BEGINNING_SPEED = SmartDashboard.getNumber("Bin Tuning/Retract Beginning Speed", BIN_RETRACT_BEGINNING_SPEED);
      BIN_EXTEND_BEGINNING_SPEED = SmartDashboard.getNumber("Bin Tuning/Extend Beginning Speed", BIN_EXTEND_BEGINNING_SPEED);
      BIN_RETRACT_RISING_SPEED = SmartDashboard.getNumber("Bin Tuning/Retract Rising Speed", BIN_RETRACT_RISING_SPEED);
      BIN_EXTEND_RISING_SPEED = SmartDashboard.getNumber("Bin Tuning/Extend Rising Speed", BIN_EXTEND_RISING_SPEED);
      BIN_BEGINNING_TRAVEL_DISTANCE = SmartDashboard.getNumber("Bin Tuning/Beginning Travel Distance", BIN_BEGINNING_TRAVEL_DISTANCE);
      BIN_RISING_TRAVEL_UP_DISTANCE = SmartDashboard.getNumber("Bin Tuning/Rising Travel Up Distance", BIN_RISING_TRAVEL_UP_DISTANCE);
      BIN_RISING_NET_CHANGE_DISTANCE = SmartDashboard.getNumber("Bin Tuning/Rising Net Change Distance", BIN_RISING_NET_CHANGE_DISTANCE);
      BIN_WAIT_TIME = SmartDashboard.getNumber("Bin Tuning/Bin Beggining Time", BIN_BEGINNING_TIME);
      USE_RISING_AGITATE = SmartDashboard.getBoolean("Bin Tuning/Use Rising", USE_RISING_AGITATE);
    }
  }

  /**
   * Constants for the pivot mechanism that changes the angle of the shooter.
   * Contains limit switch DIOs, encoder limits, and basic PID tracking for
   * targeting.
   */

  public static class ShooterPivotConstants {
    public static final int DIO_EXT_LIMIT = 4;
    public static final int DIO_RET_LIMIT = 5;
    public static final double KP = 6;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double ENC_REVS_MAX = 12.4;
    public static final double TARGET_REVS = 50;
    public static final double CONSTANT_FORWARD_SPEED = 0.1;
    public static final double CONSTANT_REVERSE_SPEED = -0.1;
  }

  /**
   * Constants for the climber mechanism.
   * Includes safety limits, target encoder revolutions for different climb
   * stages, and motor speeds.
   */

  public static class ClimberConstants {
    public static final int DIO_CLIMBER_TOP = 6;
    public static final int DIO_CLIMBER_BOTTOM = 7;
    public static final int PWM_CLIMB_LOCK = 0;
    public static final double MAX_ENCODER_REVS = 120.0;
    public static final double CLIMBER_UP_SPEED = 0.75;
    public static final double CLIMBER_DOWN_SPEED = -0.75;
    public static final double TEST_MM_REVS = 35.0;
    public static final double PREP_CLIMBER_REVS = 109.0;
    public static final double CLIMB_L1_SIDE_REVS = 109.0;
    public static final double CLIMB_L1_FRONT = 44.0;

    public static final double CLIMBER_KP = 3;
  }

  /**
   * Constants for the intake mechanism that pulls game pieces off the floor.
   * Manages speeds, current limits to prevent motor burnout, and PID tuning for
   * intelligent feed control.
   */

  public static class IntakeConstants {
    public static final double INTAKE_SPEED = 0.5;
    public static final double OUTTAKE_SPEED = -0.5;
    public static final double DISABLE_CURRENT = 30.0;
    public static final double TEST_SPEED = 0.3;
    public static final int EJECT_RPM = 3500;
    public static final int INTAKE_RPM = 5500;
    public static final double KV_I = .115;
    public static final double KP_I = 1.0;
    public static final double KI_I = 0.0;
    public static final double KD_I = 0.0;
  }

  /**
   * Constants for the internal indexer/floor mechanism that moves game pieces
   * through the robot's interior.
   */

  public static class FloorConstants {
    public static final double TEST_SPEED = -0.1;
    public static final int DESIRED_RPM = 1000;
    public static final double KV_F = 0.0;
    public static final double KP_F = 0.0;
    public static final double KI_F = 0.0;
    public static final double KD_F = 0.0;
  }

  /**
   * Constants and maps dedicated to vision targeting, Limelight configurations,
   * and physical field coordinates for autonomous scoring.
   */

  public static final class Targeting {

    public static double SHAKE_FREQUENCY = 2.4;
    public static double SHAKE_SPREAD = 15.0;
    public static double WAIT_TIME = 0.0;
    public static double END_TIME = 115.0;

    /**
     * Pushes targeting variables to the SmartDashboard for live tuning.
     */

    public static void pushToDashboard() {
      SmartDashboard.putNumber("Shooter Tuning/SHAKE_FREQUENCY", SHAKE_FREQUENCY);
      SmartDashboard.putNumber("Shooter Tuning/SHAKE_SPREAD", SHAKE_SPREAD);
      SmartDashboard.putNumber("Shooter Tuning/WAIT_TIME", WAIT_TIME);
      SmartDashboard.putNumber("Shooter Tuning/END_TIME", END_TIME);

    }

    /**
     * Pulls updated targeting variables from the SmartDashboard.
     */

    public static void pullFromDashboard() {
      SHAKE_FREQUENCY = SmartDashboard.getNumber("Shooter Tuning/SHAKE_FREQUENCY", SHAKE_FREQUENCY);
      SHAKE_SPREAD = SmartDashboard.getNumber("Shooter Tuning/SHAKE_SPREAD", SHAKE_SPREAD);
      WAIT_TIME = SmartDashboard.getNumber("Shooter Tuning/WAIT_TIME", WAIT_TIME);
      END_TIME = SmartDashboard.getNumber("Shooter Tuning/END_TIME", END_TIME);
    }

    /**
     * Defines unique network names for Limelight vision cameras.
     */

    public static final String[] CAMERA_NAMES = {
        "limelight",
        "limelight-back"
    };

    public static final double CHASSIS_WIDTH_INCHES = 27.5;
    public static final double CHASSIS_WIDTH_METERS = Units.inchesToMeters(CHASSIS_WIDTH_INCHES);
    public static final double ROBOT_WIDTH_INCHES = 33.5;
    public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(ROBOT_WIDTH_INCHES);
    public static final double BLUE_TOWER_CENTER_Y_METERS = 3.746;
    public static final double RED_TOWER_CENTER_Y_METERS = 4.324;

    public static final double AUTO_ROTATE_KP = 4.25;
    public static final double AUTO_ROTATE_KD = 0.025;
    public static final double AUTO_ROTATE_FEEDFORWARD = 0.5;
    public static final double AUTO_ROTATE_TOLERANCE = 2.0;

    public static final double AUTO_SHAKE_KP = 90.0;
    public static final double AUTO_SHAKE_KD = 0.0;

    /**
     * InterpolatingDoubleTreeMap stores key-value pairs and returns interpolated
     * values
     * for missing keys. For example, if a key of 50.0 is requested but not mapped,
     * the map calculates a value based on the surrounding keys (42.0 and 74.6).
     */

    /**
     * Maps a given distance to a required hood angle for accurate shooting.
     */

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
      hoodAngleMap.put(42.0, 3.0);
      hoodAngleMap.put(74.6, 4.75);
      hoodAngleMap.put(111.5, 6.45);
      hoodAngleMap.put(145.5, 8.5);
      hoodAngleMap.put(180.0, 11.1);
      hoodAngleMap.put(210.0, 12.1);
    }

    /**
     * Maps a given distance to a required flywheel RPM for accurate shooting.
     */

    public static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    static {
      rpmMap.put(42.0, 2300.0);
      rpmMap.put(74.6, 2410.0);
      rpmMap.put(111.5, 2490.0);
      rpmMap.put(145.5, 2545.0);
      rpmMap.put(180.0, 2690.0);
      rpmMap.put(210.0, 2800.0);
    }

    /**
     * Maps a given distance to a required pre-feeder speed.
     */

    public static final InterpolatingDoubleTreeMap preFeederMap = new InterpolatingDoubleTreeMap();

    static {
      preFeederMap.put(42.0, 2250.0);
      preFeederMap.put(74.6, 2300.0);
      preFeederMap.put(111.5, 2400.0);
      preFeederMap.put(145.5, 2500.0);
      preFeederMap.put(180.0, 2650.0);
      preFeederMap.put(210.0, 2800.0);
    }

    /**
     * Maps a given distance to an estimated flight time of the game piece.
     */

    public static final InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

    static {
      timeMap.put(42.0, 1.22);
      timeMap.put(74.6, 1.05);
      timeMap.put(111.5, 1.2);
      timeMap.put(145.5, 1.15);
      timeMap.put(180.0, 1.15);
      timeMap.put(210.0, 1.15);
    }

    /**
     * Maps neutral zone parameters for specific flywheel RPM targets.
     */

    public static final InterpolatingDoubleTreeMap nuetralMap = new InterpolatingDoubleTreeMap();

    static {
      nuetralMap.put(0.0, 2500.0);
      nuetralMap.put(75.0, 3000.0);
      nuetralMap.put(150.0, 3500.0);
      nuetralMap.put(225.0, 3750.0);
      nuetralMap.put(300.0, 4500.0);
    }

    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);
    private static Pose3d poseExample = APRIL_TAG_LAYOUT.getTagPose(10).get();

    public static final double RED_ALLIANCE_HUB_CENTER_X = 468.56;
    public static final double RED_ALLIANCE_HUB_CENTER_Y = 158.32;

    public static final double BLUE_ALLIANCE_HUB_CENTER_X = 181.56;
    public static final double BLUE_ALLIANCE_HUB_CENTER_Y = 158.32;

    public static final double RED_NEUTRAL_TOLERANCE_X = 446.36;
    public static final double BLUE_NEUTRAL_TOLERANCE_X = 203.76;

    public static final double RED_NEUTRAL_MID_RIGHT = 125;
    public static final double RED_NEUTRAL_MID_LEFT = 192;

    public static final double BLUE_NEUTRAL_MID_RIGHT = 192;
    public static final double BLUE_NEUTRAL_MID_LEFT = 125;
  }

  /**
   * PathPlanner PID tuning values.
   * These dictate how aggressively the robot corrects its position and heading
   * when following an autonomous path.
   */

  public static final class PathPlanner {
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(4, 0.0, 0.2);
  }

  /**
   * The master configuration class for the Swerve Drive base.
   * Contains physical dimensions, gear ratios, current limits, and kinematic
   * definitions required to translate
   * joystick inputs into complex omnidirectional movement.
   */

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSTalonFXSwerveConstants chosenModule =

        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
            .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_11);

    public static final double trackWidth = Units.inchesToMeters(20.5);
    public static final double wheelBase = Units.inchesToMeters(20.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double angleKP = 100.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.05;
    public static final double angleKS = 0.05;
    public static final double angleKV = 1.5;

    public static final double driveKP = 2.5;
    public static final double driveKI = 0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    public static final double driveKS = 0;
    public static final double driveKV = 0;
    public static final double driveKA = 0;

    public static final double maxSpeed = 4.5;
    public static final double throttle = 1.0;
    public static final double maxAngularVelocity = 10.0;
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /**
     * Configuration for physical Swerve Module 0.
     * Binds the specific drive motor ID, steering motor ID, and CANCoder ID
     * together.
     * Angle offset ensures the wheel points perfectly straight on boot.
     */

    public static final class Mod0 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.78);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /**
     * Configuration for physical Swerve Module 1.
     */

    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-175.34);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /**
     * Configuration for physical Swerve Module 2.
     */

    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.32);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /**
     * Configuration for physical Swerve Module 3.
     */

    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-74.44);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  /**
   * Specific speed and acceleration constraints for autonomous path following.
   */

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI;
    public static final double kPXController = 4;
    public static final double kPYController = 6;
    public static final double kPThetaController = 10;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /**
   * A mapping of standard Xbox controller buttons to integer IDs.
   * This allows code to read `Constants.XboxController.A` instead of remembering
   * that the "A" button is ID 1.
   */

  public static class XboxController {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW = 7;
    public static final int MENU = 8;
    public static final int LM = 9;
    public static final int RM = 10;

    /**
     * Maps continuous joystick axes and analog triggers to integer IDs.
     */
    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }

    /**
     * Maps the Directional Pad (D-Pad) positions to their respective degree angles.
     */
    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
  }
}