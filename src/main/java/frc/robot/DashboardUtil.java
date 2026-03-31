// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class DashboardUtil {

    private static HashMap<Double, String> shiftToName = new HashMap<>();
    
    static {
        shiftToName.put(130.0, "TRANSITION SHIFT");
        shiftToName.put(105.0, "SHIFT 1");
        shiftToName.put(80.0, "SHIFT 2");
        shiftToName.put(55.0, "SHIFT 3");
        shiftToName.put(30.0, "SHIFT 4");
        shiftToName.put(0.0, "ENDGAME");
    }

    private static double[] shiftTimes = new double[] {
        130.0, 105.0, 80.0, 55.0, 30.0, 0.0
    };

    private static Field2d autoField = new Field2d();

    static {
        SmartDashboard.putData("Auto Field", autoField);
    }

    public static double getShiftTime() {
        double currentMatchTime = DriverStation.getMatchTime();
        for (double shiftTime : shiftTimes) {
            if (shiftTime < currentMatchTime) {
                return currentMatchTime - shiftTime;
            }
        }

        return -1.0;
    }

    public static String getCurrentShift() {
        double currentMatchTime = DriverStation.getMatchTime();
        for (double shiftTime : shiftTimes) {
            if (shiftTime < currentMatchTime) {
                return shiftToName.get(shiftTime);
            }
        }

        return "NONE";
    }

    private static void updateAutoFieldDisplay(Command autoCommand) {
        Field2d field = (Field2d) SmartDashboard.getData("Auto Field");
        var traj = field.getObject("autoTrajectory");
        var alliance = DriverStation.getAlliance();

        field.setRobotPose(new Pose2d());
        traj.setPoses(new Pose2d());

        if (autoCommand == null || autoCommand.equals(Commands.none()) || autoCommand == null) {
            return; 
        }
        
        if (!(autoCommand instanceof PathPlannerAuto)) {
            System.out.println("autoCommand is not PathPlannerAuto, will not display to Auto Field on dashboard");
            return;
        }

        var auto = (PathPlannerAuto) autoCommand;
        String name = auto.getName();
        if (name == null || name.isBlank()) {
            System.out.println("PathPlannerAuto has null or blank name, cannot display to dashboard");
        }

        try {
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName());
            
            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for (PathPlannerPath path : paths) {
                for (Pose2d pose : path.getPathPoses()) {
                    poses.add(alliance.isPresent() && alliance.get() == Alliance.Red ? FlippingUtil.flipFieldPose(pose) : pose);
                }
            }
            
            Pose2d robotPose = alliance.isPresent() && alliance.get() == Alliance.Red ? FlippingUtil.flipFieldPose(auto.getStartingPose()) : auto.getStartingPose();
            field.setRobotPose(robotPose);
            traj.setPoses(poses);

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("exception getting paths from auto files, when displaying auto to field on dashboard");
        }
    }

    public static void putAutoToField() {
        
        try {
            var autoChooser = ((SendableChooser<Command>) SmartDashboard.getData("Auto Routine"));

            if (autoChooser == null) { return; }
            
            updateAutoFieldDisplay(autoChooser.getSelected());
            autoChooser.onChange(DashboardUtil::updateAutoFieldDisplay);
            
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("exception trying to display auto on Field2d on dashboard");
        }   
    }
}
