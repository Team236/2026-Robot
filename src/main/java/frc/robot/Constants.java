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

// -------------------------------------------------------------------------------------------
// The constants file is contains numeric values for different variables that can be accessed.
// This prevents hardcoded values (magic numbers) in subsystem / command files, minimizes
// error, and simplifies tuning by allowing robot variables to be updated in one place.
// -------------------------------------------------------------------------------------------

public final class Constants {
  public static final double stickDeadband = 0.1;

  // -------------------------------------------------------------------------------------------
  // Within the Constants "class" the varaibles are organized into more descriptive classes. 
  // This makes it easier to locate specific variables. For example the Controller class holds
  // the value of the different drive controllers. (See lines 35 to 38)
  // -------------------------------------------------------------------------------------------

  public static final class Controller {
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;
  }

  // -------------------------------------------------------------------------------------------
  // To control a motor it must have a unique ID associated with it. This allows the computing
  // system to send and recieve data from that motor. (See lines 48 to 55)
  // -------------------------------------------------------------------------------------------

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

  // -------------------------------------------------------------------------------------------
  // PIDF controlers are a common way to control a motor and use feedback. It stands for 
  // proportinal, integral, and dirivative. The Kv is an initial baseline voltage, Kp is a 
  // corrective force based on current error, Ki is an accumulative error to eliminates steady 
  // state error by accumulating past error over time, Kd is a "breaking force" as the motor 
  // approaches its target to prevent overshoot. Their numeric values are stored within this
  // constants file and are used in subsystems. (See lines 70 to 73 & 80 to 83)
  // -------------------------------------------------------------------------------------------

  public static class ShooterConstants {
    public static final double MAIN_MOTOR_RPM = 2550;
    public static final double MAIN_MOTOR_SPEED = 0.3;
    public static final double KV_MAIN = 0.122;
    public static final double KP_MAIN = 0.63;
    public static final double KI_MAIN = 0;
    public static final double KD_MAIN = 0;
  }

  public static class PreFeederConstants {
    public static final int DIO_COUNTER = 10;
    public static final double TEST_SPEED = 0.3;
    public static final int DESIRED_RPM = 2550;
    public static final double KV_PF = 0.0983;
    public static final double KP_PF = 0.97;
    public static final double KI_PF = 0;
    public static final double KD_PF = 0;
  }

  // -------------------------------------------------------------------------------------------
  // The "BinReleaseConstants" class contains many numeric values including DIO variables. DIO
  // stands for Digital Input Output. DIO's are sensors using booleans. In this case the DIO's
  // are used to know if a bin mechanism is able to continue to move foward or backwards. These
  // values "8" and "9" tell the code what pins to read on the Robo Rio. (See lines 95 to 96)
  // -------------------------------------------------------------------------------------------

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

  // -------------------------------------------------------------------------------------------
  // Similar to the DIOs used in the "BinReleaseConstants" class, motor revs, revolutions, can
  // be monitored to know where a mechanism is relative to its starting possitition. These revs
  // also serve as target positions for a PIDF controller. (See lines 106, 109, & 114 to 116)
  // -------------------------------------------------------------------------------------------

  // ----------------------------------------------------------------------------------------------
  // The "ChangableBinConstants" class allows for changable values of variables through a dashboard.
  // This improves testing time as new values can be evaluated without editing code and redeploying.
  // (See lines 131 to 184)
  // ----------------------------------------------------------------------------------------------

  public static class ChangableBinConstants {

    // ----------------------------------------------------------------------------------------------
    // Initial numeric values are set to each variable (See line 137 to 147)
    // ----------------------------------------------------------------------------------------------

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

    // ----------------------------------------------------------------------------------------------
    // The method "pushToDashboard()" sends the current values of each variable (See line 153 to 165)
    // ----------------------------------------------------------------------------------------------

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

    // ----------------------------------------------------------------------------------------------
    // The method "pullFromDashboard()" gets the current number assigned to each of the dashboard
    // elements and updates the varaibles in the "ChangableBinConstants" class. (See line 172 to 183)
    // ----------------------------------------------------------------------------------------------

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

  // ----------------------------------------------------------------------------------------------
  // The "FORWARD_SPEED" and "REVERSE_SPEED" contian a number -1 to 1 instructing the computer how
  // much voltage should be supplied to a motor, where -1 is 100% reverse and 1 is 100% forward.
  // (See lines 200 to 201)
  // ----------------------------------------------------------------------------------------------

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

  // ----------------------------------------------------------------------------------------------
  // The constants classes below are similar to the ones explained before (See lines 204 to 243)
  // ----------------------------------------------------------------------------------------------

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

  public static class FloorConstants {
    public static final double TEST_SPEED = -0.1;
    public static final int DESIRED_RPM = 1000;
    public static final double KV_F = 0.0;
    public static final double KP_F = 0.0;
    public static final double KI_F = 0.0;
    public static final double KD_F = 0.0;
  }
  
  // ----------------------------------------------------------------------------------------------
  // The "Targeting" class contains similar code to "ChangableBinConstants" (See lines 251 to 269)
  // ----------------------------------------------------------------------------------------------

  public static final class Targeting {

    public static double SHAKE_FREQUENCY = 2.4;
    public static double SHAKE_SPREAD = 15.0;
    public static double WAIT_TIME = 0.0;
    public static double END_TIME = 115.0;

    public static void pushToDashboard() {
      SmartDashboard.putNumber("Shooter Tuning/SHAKE_FREQUENCY", SHAKE_FREQUENCY);
      SmartDashboard.putNumber("Shooter Tuning/SHAKE_SPREAD", SHAKE_SPREAD);
      SmartDashboard.putNumber("Shooter Tuning/WAIT_TIME", WAIT_TIME);
      SmartDashboard.putNumber("Shooter Tuning/END_TIME", END_TIME);

    }

    public static void pullFromDashboard() {
      SHAKE_FREQUENCY = SmartDashboard.getNumber("Shooter Tuning/SHAKE_FREQUENCY", SHAKE_FREQUENCY);
      SHAKE_SPREAD = SmartDashboard.getNumber("Shooter Tuning/SHAKE_SPREAD", SHAKE_SPREAD);
      WAIT_TIME = SmartDashboard.getNumber("Shooter Tuning/WAIT_TIME", WAIT_TIME);
      END_TIME = SmartDashboard.getNumber("Shooter Tuning/END_TIME", END_TIME);
    }

    // ----------------------------------------------------------------------------------------------
    //  The "CAMERA_NAMES" array defines 2 unique camera names for limelights (See lines 275 to 278)
    // ----------------------------------------------------------------------------------------------

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

    // ----------------------------------------------------------------------------------------------
    // The use of "InterpolatingDoubleTreeMap" allows for data to be stored and later pulled from.
    // These are usefull as they return values for any key passed in. A key that of 200 might not be
    // entered however an interpolation will be performed and a value returned. (See lines 301 to 353)
    // ----------------------------------------------------------------------------------------------

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
      hoodAngleMap.put(42.0, 3.0);
      hoodAngleMap.put(74.6, 4.75);
      hoodAngleMap.put(111.5, 6.45);
      hoodAngleMap.put(145.5, 8.5);
      hoodAngleMap.put(180.0, 11.1);
      hoodAngleMap.put(210.0, 12.1);
    }

    public static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    static {
      rpmMap.put(42.0, 2300.0);
      rpmMap.put(74.6, 2410.0);
      rpmMap.put(111.5, 2490.0);
      rpmMap.put(145.5, 2545.0);
      rpmMap.put(180.0, 2690.0);
      rpmMap.put(210.0, 2800.0);
    }

    public static final InterpolatingDoubleTreeMap preFeederMap = new InterpolatingDoubleTreeMap();

    static {
      preFeederMap.put(42.0, 2250.0);
      preFeederMap.put(74.6, 2300.0);
      preFeederMap.put(111.5, 2400.0);
      preFeederMap.put(145.5, 2500.0);
      preFeederMap.put(180.0, 2650.0);
      preFeederMap.put(210.0, 2800.0);
    }

    public static final InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

    static {
      timeMap.put(42.0, 1.22);
      timeMap.put(74.6, 1.05);
      timeMap.put(111.5, 1.2);
      timeMap.put(145.5, 1.15);
      timeMap.put(180.0, 1.15);
      timeMap.put(210.0, 1.15);
    }

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

    // ----------------------------------------------------------------------------------------------
    // Field coordinates can also be stored in Constants. (See lines 363 to 376)
    // ----------------------------------------------------------------------------------------------

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

    // ----------------------------------------------------------------------------------------------
    // Pathplanner uses PID controllers. These constants are stored below. (See lines 384 to 385)
    // ----------------------------------------------------------------------------------------------

  public static final class PathPlanner {
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(4, 0.0, 0.2);
  }

    // ----------------------------------------------------------------------------------------------
    // The Constants file holds many values, critical for many systems. The code below stores values
    // that the swerve code uses during its calculations. Try to understand the code bellow on your
    // own using the information you have already learned. (See lines 394 to 489)
    // ----------------------------------------------------------------------------------------------

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSTalonFXSwerveConstants chosenModule =

        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
            .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_11);

    public static final double trackWidth = Units.inchesToMeters(20.5);
    public static final double wheelBase = Units.inchesToMeters(20.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    // ----------------------------------------------------------------------------------------------
    // Translation2d is a method used to store cordinates in one object. (See lines 411 to 414)
    // ----------------------------------------------------------------------------------------------

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

    // ----------------------------------------------------------------------------------------------
    // It is not necessary to understand the exact definitions of all constants. Rather understanding
    // the idea of a global set of values interacting with all systems in far more inportant. 
    // ----------------------------------------------------------------------------------------------

    // ----------------------------------------------------------------------------------------------
    // Mod, or modules, refer to the physical swerve modules we use. Mod0 contains  a drive and 
    // stearing motor. These hardware components are defined below. The "angleOffset" constant
    // provides an offset value to help the wheels auto rotate straight when enableing the robot.
    // (See lines 470 to 504)
    // ----------------------------------------------------------------------------------------------

    public static final class Mod0 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(40.78);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-175.34);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.32);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-74.44);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  // ----------------------------------------------------------------------------------------------
  // The "AutoConstants" class contains set values specific to this years robot. These can be later
  // used for pathplanner or path creations. (See lines 512 to 522)
  // ----------------------------------------------------------------------------------------------

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

  // ----------------------------------------------------------------------------------------------
  // The "XboxController" maps button, axis, and POV constants corresponding to standard Xbox 
  // controller mappings. (See lines 530 to 557)
  // ----------------------------------------------------------------------------------------------

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

    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }

    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
  }
}