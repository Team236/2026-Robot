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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 * constants are needed, to reduce verbosity.
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
    public static final int ID_BIN_REL = 8; 
    public static final int ID_INTAKE = 13; //9
    public static final int ID_FLOOR = 10; 
    public static final int ID_SHOOTER_LEFT_MAIN = 11;
    public static final int ID_SHOOTER_RIGHT_MAIN = 12;
    public static final int ID_SHOOTER_LEFT_TOP = 23; //13
    public static final int ID_SHOOTER_RIGHT_TOP = 14;
    public static final int ID_SHOOTER_PIVOT = 15;
    public static final int ID_CLIMBER = 16; 
  }

  public static class Shooter {
    public static final double MAIN_MOTOR_RPM = 2250;
    public static final double TOP_MOTOR_RPM = 2000;

    public static final double MAIN_MOTOR_SPEED = -0.5;
    public static final double TOP_MOTOR_SPEED = -0.1;

    // TODO find final gains
    public static final double KV_MAIN = 0.116; // feed forward
    public static final double KP_MAIN = 0.4;
    public static final double KI_MAIN = 0;
    public static final double KD_MAIN = 0; 

    public static final double KV_TOP = 0.116; // feed forward
    public static final double KP_TOP = 0.4;
    public static final double KI_TOP = 0;
    public static final double KD_TOP = 0;
  }

  public static final class Bin {
    public static final double KP = 0.2;  
    public static final double KI = 0;
    public static final double KD = 0;
    public static final int DIO_EXT_LIMIT = 8;
    public static final int DIO_RET_LIMIT = 9;
    public static final int ENC_REVS_MAX = 1000; //TBD
    public static final double MANUAL_EXT_SPEED = 0.1;
    public static final double MANUAL_RET_SPEED = -0.1;
    public static final double POSITION1 = 50;
  }

  public static class Pivot {
    public static final int DIO_EXT_LIMIT = 4;
    public static final int DIO_RET_LIMIT = 5;
    public static final double KP = 0.16; 
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double ENC_REVS_MAX = 1000; //TODO find actual
    public static final double TARGET_REVS = 200;
    public static final double CONSTANT_SPEED_TEST_VALUE = 0.1;
  }

  public static class Climb {
    public static final int DIO_CLIMBER_TOP = 6;
    public static final int DIO_CLIMBER_BOTTOM = 7;

    public static final double MAX_ENCODER_REVS = 1000.0; // MotionMagic/PID stops if here; DO NOT PID CLIMB HIGHER 
    public static final double CLIMBER_SPEED_TEST = 0.1;
    public static final double TEST_MM_REVS = 0.0;
  }

  public static  class Intake {
    public static final double INTAKE_SPEED = 0.1;
    public static final double OUTTAKE_SPEED = -0.1;
  }  

  public static class FloorC {
    public static final double TEST_SPEED = -0.1;
  }

  public static final class Targeting {
    //Use these do MetricDriveFwdSideDist field centric robot to tag 
    public static final double DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER = 18.25; // inches
    public static final double DIST_ROBOT_CENTER_TO_LL_SIDEWAYS = 8; //
    //use this with TargetPose-CameraSpace: inches
    public static final double DIST_FORWARDS_CAMERA_TO_FRAME = 5.2;
    public static final double BUMPER_THICKNESS = 3.25;
    public static final double DIST_CAMERA_TO_BUMPER_FWD = BUMPER_THICKNESS + DIST_FORWARDS_CAMERA_TO_FRAME;

    public static final double KP_ROTATION = 0.008; //kP value for rotation
    public static final double KP_TRANSLATION = 0.4;//kP value for forward (translation) motion
    public static final double KP_STRAFE = 0.9;// 0.475;  //kP value for the sideways (strafe) motio%n 

    public static final double DONT_SEE_TAG_WAIT_TIME = 0.3;
    public static final double POSE_VALIDATION_TIME = 2; //TODO - shorten
}

public static final class PathPlanner { //TODO -- UPDATE TO NEW ROBOT
  public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(4.5, 0.0, 0.0); // seems like these affect movement of robot when it tries to correct itself (goes off path), not for just regular movement
  public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0); // ^^^^
}

public static final class Swerve { //TODO -- UPDATE ALL VALUES / TUNE
        public static final int pigeonID = 1; //gryo

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);
      
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5); //2024 testbed //TODO MUST BE UPDATED
        public static final double wheelBase = Units.inchesToMeters(23.5); //2024 testbed
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
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

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
        public static final class Mod0 { //TODO: This must be tuned to specific robot TUNE
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.84);//(81.1+180); TESTBED//-119.79; old -120.15
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* FRONT RIGHT Module - Module 1 */
            public static final class Mod1 { //TODO: This must be tuned to specific robot TUNE
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(159.61);//(-20.83+180)Estbed; 130.87;old 131.045
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); 
        }    
        /* BACK LEFT Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot TUNE
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-172.62);//(8.1+180); TESTBED //106.35; old 106.87
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* BACK RIGHT Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot TUNE
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-99.84);//(-17.75+180);  TESTBED//130.87; old 130.95
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