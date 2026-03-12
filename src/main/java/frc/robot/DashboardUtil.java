// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

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
        if (DriverStation.getMatchType() == MatchType.None) {
            return -1.0;
        }

        double currentMatchTime = DriverStation.getMatchTime();
        for (double shiftTime : shiftTimes) {
            if (shiftTime < currentMatchTime) {
                return currentMatchTime - shiftTime;
            }
        }

        return -1.0;
    }
}
