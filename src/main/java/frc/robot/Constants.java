// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.F
 */
public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Controller {
   //USB port number of the xbox controllers
   public static final int USB_DRIVECONTROLLER = 0;
   public static final int USB_AUXCONTROLLER = 1;
  }

  //AUTO SWITCHES
  public static final int DIO_AUTO_1 = 0;
  public static final int DIO_AUTO_2 = 1;
  public static final int DIO_AUTO_3 = 2;
  public static final int DIO_AUTO_4 = 3;

  public static class MotorControllers {
    public static final int SMART_CURRENT_LIMIT = 40;
    //MOTOR ID NUMBERS (for swerve drive/steer motors - see Swerve below):
    public static final int ID_BIN_REL = 8; //8
    public static final int ID_INTAKE_LEFT = 9; 
    public static final int ID_FLOOR = 10; 
    public static final int ID_SHOOTER_LEFT_MAIN = 12; // 11
    public static final int ID_SHOOTER_RIGHT_MAIN = 11; // 12
   // public static final int ID_SHOOTER_MID_MAIN = 18;
   // public static final int ID_SHOOTER_LEFT_TOP = 16;
   // public static final int ID_SHOOTER_RIGHT_TOP = 17;
    public static final int ID_SHOOTER_PIVOT = 15;
    public static final int ID_CLIMBER = 13; //13
    public static final int ID_PRE_FEEDER = 14; //14
  }

  public static class ShooterConstants {
    public static final double MAIN_MOTOR_RPM = 2550;
    public static final double MAIN_MOTOR_SPEED = 0.3;
    public static final double KV_MAIN = 0.122; // feed forward
    public static final double KP_MAIN = 0.63;
    public static final double KI_MAIN = 0;
    public static final double KD_MAIN = 0; 

    //public static final double TOP_MOTOR_RPM = 2100;
    //public static final double TOP_MOTOR_SPEED = -0.5;
    //public static final double KV_TOP = 0.117; // feed forward
    //public static final double KP_TOP = 0.4;
    //public static final double KI_TOP = 0;
    //public static final double KD_TOP = 0;
  }
  
  public static class PreFeederConstants {
    public static final int DIO_COUNTER = 10;
    public static final double TEST_SPEED = 0.3;//constant speed for testing
    public static final int DESIRED_RPM = 2550; //3150;
    public static final double KV_PF = 0.0983; // feed forward
    public static final double KP_PF = 0.97;
    public static final double KI_PF = 0;
    public static final double KD_PF = 0;
  }

  public static final class BinReleaseConstants {

    public static final int DIO_EXT_LIMIT = 8;//DIO port number on RoboRio
    public static final int DIO_RET_LIMIT = 9; //DIO port number on RoboRio
    public static final double KP_BIN = 0.2;  
    public static final double KI_BIN = 0;
    public static final double KD_BIN = 0;
    public static final int ENC_REVS_MAX = 1000; //TBD
    public static final double MANUAL_EXT_SPEED = 0.1;
    public static final double MANUAL_RET_SPEED = -0.1;
    public static final double POSITION1 = 50;
    public static final double AGITATE_POSITION = 7.0; // revs
    public static final double AGITATE_TOLERANCE = 0.5; // revs
    public static final double AGITATE_TEST_SPEED = 0.2; // seconds for one full cycle
  }

  public static class ShooterPivotConstants {
    public static final int DIO_EXT_LIMIT = 4;//DIO port number on RoboRio
    public static final int DIO_RET_LIMIT = 5;//DIO port number on RoboRio
    public static final double KP = 6; 
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double ENC_REVS_MAX = 11.2; //TODO find actual
    public static final double TARGET_REVS = 50;
    public static final double CONSTANT_FORWARD_SPEED = 0.06;
    public static final double CONSTANT_REVERSE_SPEED = -0.06;
  }

  public static class ClimberConstants {
    public static final int DIO_CLIMBER_TOP = 6; //DIO port number on RoboRio
    public static final int DIO_CLIMBER_BOTTOM = 7; //DIO port number on RoboRio
    public static final int PWM_CLIMB_LOCK = 0;  //PWM port number on RoboRio
    public static final double MAX_ENCODER_REVS = 120.0;//TBD MotionMagic/PID, stops if here; DO NOT PID CLIMB HIGHER 
    public static final double CLIMBER_UP_SPEED = 0.75;
    public static final double CLIMBER_DOWN_SPEED = -0.75;
    public static final double TEST_MM_REVS = 35.0;
    public static final double PREP_CLIMBER_REVS = 109.0;
    public static final double CLIMB_L1_SIDE = 50.0;
    public static final double CLIMB_L1_FRONT = 44.0;

    public static final double CLIMBER_KP = 3;

    /*
     * blue tower left (driver perspective):
     *   55.0, 129.0 according to robot? cad says 57.32, 130
     *   
     * 
     */
      
  }

  public static  class IntakeConstants {
    public static final double INTAKE_SPEED = 0.5;
    public static final double OUTTAKE_SPEED = -0.5;
    //these next variables are for the intake pid command, but they are ripped straight from the prefeeder pid, so they may not be right.
    //TODO: TUNE THESE VALUES FOR INTAKE PID COMMAND
    public static final double TEST_SPEED = 0.3;//constant speed for testing
    public static final int DESIRED_RPM = 3150;
    public static final double KV_I = 0.0; //0.0975; // feed forward
    public static final double KP_I = 0.0; //0.4;
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

  public static final class Targeting { //UPDATE FOR 2026 ROBOT 
    //Use these do MetricDriveFwdSideDist field centric robot to tag 
    // public static final double DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER = 18.25; // inches
    // public static final double DIST_ROBOT_CENTER_TO_LL_SIDEWAYS = 8; //
    //use this with TargetPose-CameraSpace: inches
    // public static final double DIST_FORWARDS_CAMERA_TO_FRAME = 5.2;
    // public static final double BUMPER_THICKNESS = 3.25;
    // public static final double DIST_CAMERA_TO_BUMPER_FWD = BUMPER_THICKNESS + DIST_FORWARDS_CAMERA_TO_FRAME;
    // public static final double KP_ROTATION = 0.008; //kP value for rotation
    // public static final double KP_TRANSLATION = 0.4;//kP value for forward (translation) motion
    // public static final double KP_STRAFE = 0.9;// 0.475;  //kP value for the sideways (strafe) motio%n 

    public static final double AUTO_ROTATE_KP = 6;
    public static final double AUTO_ROTATE_KD = 0.05;
    public static final double AUTO_ROTATE_FEEDFORWARD = 0.5;
    public static final double AUTO_ROTATE_TOLERANCE = 1; //degrees

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
      hoodAngleMap.put(42.0, 2.83); 
      hoodAngleMap.put(74.6, 4.36);
      hoodAngleMap.put(111.5, 6.36);
      hoodAngleMap.put(145.5, 8.41);
      hoodAngleMap.put(180.0, 10.94);
    }

    public static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    static {
      rpmMap.put(42.0, 2150.0); 
      rpmMap.put(74.6, 2200.0);
      rpmMap.put(111.5, 2300.0);
      rpmMap.put(145.5, 2400.0);
      rpmMap.put(180.0, 2550.0);
    }

    public static final InterpolatingDoubleTreeMap preFeederMap = new InterpolatingDoubleTreeMap();

    static {
      preFeederMap.put(42.0, 2150.0); 
      preFeederMap.put(74.6, 2200.0);
      preFeederMap.put(111.5, 2300.0);
      preFeederMap.put(145.5, 2400.0);
      preFeederMap.put(180.0, 2550.0);
    }

    public static final InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

    static {
      timeMap.put(42.0, 1.22); 
      timeMap.put(74.6, 1.05);
      timeMap.put(111.5, 1.2);
      timeMap.put(145.5, 1.15);
      timeMap.put(180.0, 1.15);
    }
    

    // public static Map<Integer, Pose2d> ID_TO_POSE = new HashMap<>();
    
    // Turns out a class already exists to help us get poses for tags if needed (below)
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark); //should this be andymark or welded?
    private static Pose3d poseExample = APRIL_TAG_LAYOUT.getTagPose(10).get(); // example for id 10

    // 
    // static {

    //   Constants.Targeting.ID_TO_POSE.put(1, new Pose2d(Units.inchesToMeters(467.08), Units.inchesToMeters(291.79), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(2, new Pose2d(Units.inchesToMeters(468.56), Units.inchesToMeters(182.08), new Rotation2d(Units.degreesToRadians(90))));
    //   Constants.Targeting.ID_TO_POSE.put(3, new Pose2d(Units.inchesToMeters(444.80), Units.inchesToMeters(172.32), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(4, new Pose2d(Units.inchesToMeters(444.80), Units.inchesToMeters(158.32), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(5, new Pose2d(Units.inchesToMeters(468.56), Units.inchesToMeters(134.56), new Rotation2d(Units.degreesToRadians(270))));
    //   Constants.Targeting.ID_TO_POSE.put(6, new Pose2d(Units.inchesToMeters(467.08), Units.inchesToMeters(24.85), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(7, new Pose2d(Units.inchesToMeters(470.03), Units.inchesToMeters(24.85), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(8, new Pose2d(Units.inchesToMeters(482.56), Units.inchesToMeters(134.56), new Rotation2d(Units.degreesToRadians(270))));
    //   Constants.Targeting.ID_TO_POSE.put(9, new Pose2d(Units.inchesToMeters(492.33), Units.inchesToMeters(144.32), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(10, new Pose2d(Units.inchesToMeters(492.33), Units.inchesToMeters(158.32), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(11, new Pose2d(Units.inchesToMeters(482.56), Units.inchesToMeters(182.08), new Rotation2d(Units.degreesToRadians(90))));
    //   Constants.Targeting.ID_TO_POSE.put(12, new Pose2d(Units.inchesToMeters(470.03), Units.inchesToMeters(291.79), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(13, new Pose2d(Units.inchesToMeters(649.58), Units.inchesToMeters(291.02), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(14, new Pose2d(Units.inchesToMeters(649.58), Units.inchesToMeters(274.02), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(15, new Pose2d(Units.inchesToMeters(649.57), Units.inchesToMeters(169.78), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(16, new Pose2d(Units.inchesToMeters(649.57), Units.inchesToMeters(152.78), new Rotation2d(Units.degreesToRadians(180))));

    //   Constants.Targeting.ID_TO_POSE.put(17, new Pose2d(Units.inchesToMeters(183.03), Units.inchesToMeters(24.85), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(18, new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(134.56), new Rotation2d(Units.degreesToRadians(270))));
    //   Constants.Targeting.ID_TO_POSE.put(19, new Pose2d(Units.inchesToMeters(205.32), Units.inchesToMeters(144.32), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(20, new Pose2d(Units.inchesToMeters(205.32), Units.inchesToMeters(158.32), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(21, new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(182.08), new Rotation2d(Units.degreesToRadians(90))));
    //   Constants.Targeting.ID_TO_POSE.put(22, new Pose2d(Units.inchesToMeters(183.03), Units.inchesToMeters(291.79), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(23, new Pose2d(Units.inchesToMeters(180.08), Units.inchesToMeters(291.79), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(24, new Pose2d(Units.inchesToMeters(167.56), Units.inchesToMeters(182.08), new Rotation2d(Units.degreesToRadians(90))));
    //   Constants.Targeting.ID_TO_POSE.put(25, new Pose2d(Units.inchesToMeters(157.79), Units.inchesToMeters(172.32), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(26, new Pose2d(Units.inchesToMeters(157.79), Units.inchesToMeters(158.32), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(27, new Pose2d(Units.inchesToMeters(167.56), Units.inchesToMeters(134.56), new Rotation2d(Units.degreesToRadians(270))));
    //   Constants.Targeting.ID_TO_POSE.put(28, new Pose2d(Units.inchesToMeters(180.08), Units.inchesToMeters(24.85), new Rotation2d(Units.degreesToRadians(180))));
    //   Constants.Targeting.ID_TO_POSE.put(29, new Pose2d(Units.inchesToMeters(.54), Units.inchesToMeters(25.62), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(30, new Pose2d(Units.inchesToMeters(.54), Units.inchesToMeters(42.62), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(31, new Pose2d(Units.inchesToMeters(.55), Units.inchesToMeters(146.86), new Rotation2d(Units.degreesToRadians(0))));
    //   Constants.Targeting.ID_TO_POSE.put(32, new Pose2d(Units.inchesToMeters(.55), Units.inchesToMeters(163.86), new Rotation2d(Units.degreesToRadians(0))));
        
    // }

    // this is from subtracting opposing April Tags on https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf find actual values in cad
    public static final double RED_ALLIANCE_HUB_CENTER_X = 468.56;
    public static final double RED_ALLIANCE_HUB_CENTER_Y = 158.32;

    public static final double BLUE_ALLIANCE_HUB_CENTER_X = 181.56;
    public static final double BLUE_ALLIANCE_HUB_CENTER_Y = 158.32;
    
}

public static final class PathPlanner { //TODO -- UPDATE TO NEW ROBOT
  public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(10, 0.0, 0.0); // seems like these affect movement of robot when it tries to correct itself (goes off path), not for just regular movement
  public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(4, 0.0, 0.2); // ^^^^
}

public static final class Swerve { //TODO -- UPDATE ALL VALUES / TUNE
        public static final int pigeonID = 1; //gryo

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO:Find for specific swerve module
        //TODO:  X2_10 has 10 tooth, 6.56/1,  X2_11 5.96/1, X2_12 5.46/1;
        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_11);//TODO - X2
        // COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);
        //SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);
      
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.5); //23.5 in 2025 //TODO MUST BE UPDATED
        public static final double wheelBase = Units.inchesToMeters(20.5); //23.5 in 2025
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve 
         * Since these wheel positions are declared relative to the physical center of the robot, the center is 0, 0,
         * or in other words, its pose is it physical center */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25; //TODO check compared to last year
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 100.0; //100.0
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.05;
        public static final double angleKS = 0.1;
        public static final double angleKV = 1.5;

        /* Drive Motor PID Values */    
        public static final double driveKP = 2.5; //0.5, 1 //TODO: This must be tuned to specific robot, default is 0.1
        public static final double driveKI = 0; //2
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; 

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0; //0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 0; //1.51;
        public static final double driveKA = 0; //.27; 

        /* Swerve Profiling Values, Meters per Second*/
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        public static final double throttle = 1.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /* FRONT LEFT Module - Module 0 */
        public static final class Mod0 { //TODO: These must be changed for each specific robot!!!
            public static final int driveMotorID = 7;//testbed may be different!!!!! old 7
            public static final int angleMotorID = 6;//testbed may be different!!!!! old 6
            public static final int canCoderID = 3;//testbed may be different!!!!! old 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(37.26); //(-146.33);//(81.1+180); TESTBED//-119.79; old -120.15
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* FRONT RIGHT Module - Module 1 */
            public static final class Mod1 { //TODO: These must be changed for each specific robot!!!
            public static final int driveMotorID = 3;//testbed may be different!!!!!
            public static final int angleMotorID = 2;//testbed may be different!!!!!
            public static final int canCoderID = 1;//testbed may be different!!!!!
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-173.49); //(10.01);//(-20.83+180)Estbed; 130.87;old 131.045
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); 
        }    
        /* BACK LEFT Module - Module 2 */
        public static final class Mod2 { //TODO: These must be changed for each specific robot!!!
            public static final int driveMotorID = 5;//testbed may be different!!!!!
            public static final int angleMotorID = 4;//testbed may be different!!!!!
            public static final int canCoderID = 2;//testbed may be different!!!!!
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(66.97); //(-117.42);//(8.1+180); TESTBED //106.35; old 106.87
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* BACK RIGHT Module - Module 3 */
        public static final class Mod3 { //TODO: These must be changed for each specific robot!!!
            public static final int driveMotorID = 1;//testbed may be different!!!!!
            public static final int angleMotorID = 0;//testbed may be different!!!!!
            public static final int canCoderID = 0;//testbed may be different!!!!!
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-73.91); //(105.9);//(-17.75+180);  TESTBED//130.87; old 130.95
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: these are for example auto - must be tuned to specific robot TUNE
        public static final double kMaxSpeedMetersPerSecond = 5.0; //4 //2.5
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0; //4 //2.5
        public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;
        //X = forward, Y = to the left, Theta positive CCW for swerve
        public static final double kPXController = 4; //4//  12 
        public static final double kPYController = 6;//7;//12; //6//TODO: RETUNE!
        public static final double kPThetaController = 10; //TODO: RETUNE!
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
  
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




/*
 * ROBO STUFF YAY
 * 
 * Width: 27.5in, 0.6985m
 * Length: 27.5in, 0.6985m
 * LLForward: -8.5in, -0.2159m
 * LLSideways: 0in, 0m
 * LLUp: 20.5in, 0.5207m
 * LLPitch: 24deg
 */