// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

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

/** Add your docs here. */
public class DashboardUtil {
    private static double[] shiftTimes = 
    {
        130.0,
        105.0,
        80.0,
        55.0,
        30.0,
        0.0
    };

    public static double getShiftTime() {
        // if (!DriverStation.isTeleop()) {
        //     return -1.0;
        // }

        double currentMatchTime = DriverStation.getMatchTime();
        for (double shiftTime : shiftTimes) {
            if (shiftTime < currentMatchTime) {
                return currentMatchTime - shiftTime;
            }
        }

        return -1.0;
    }

    public static void putAutoToField() {
        Field2d field = (Field2d) SmartDashboard.getData("Field");
        var traj = field.getObject("autoTrajectory");
        var alliance = DriverStation.getAlliance();

        if (DriverStation.isTeleop() || DriverStation.isTeleopEnabled()) {
            return;
        }

        try {
            PathPlannerAuto auto = AutoSwitchHelpers.getPathPlannerAuto();
            auto = auto == null ? (PathPlannerAuto) ((SendableChooser) SmartDashboard.getData("Auto Routine")).getSelected() : auto;
            
            if (auto == null) { return; }

            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName());

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for (PathPlannerPath path : paths) {
                for (Pose2d pose : path.getPathPoses()) {
                    poses.add(alliance.isPresent() && alliance.get() == Alliance.Red ? FlippingUtil.flipFieldPose(pose) : pose);
                }
            }
            
            traj.setPoses(poses);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("exception trying to display auto on Field2d on dashboard");
        }   
    }
}
