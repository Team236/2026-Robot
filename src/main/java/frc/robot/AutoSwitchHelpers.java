package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoSwitchHelpers {

    private static DigitalInput autoSwitch1;
    private static DigitalInput autoSwitch2;
    private static DigitalInput autoSwitch3;
    private static DigitalInput autoSwitch4;

    private static HashMap<Integer, PathPlannerAuto> autoMap = new HashMap<>();

    static 
    {
        autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
        autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
        autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
        autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);
    }

    private AutoSwitchHelpers() {}

    public static void put(boolean s1, boolean s2, boolean s3, boolean s4, PathPlannerAuto pathPlannerAuto) 
    {
        boolean[] input = {s1, s2, s3, s4};
        int mask = convertToMask(input);

        autoMap.put(mask, pathPlannerAuto);
    }

    public static void put(int mask, PathPlannerAuto pathPlannerAuto)
    {
        if (mask < 0 || mask >= 16)
        {
            return;
        }

        autoMap.put(mask, pathPlannerAuto);
    }

    public static PathPlannerAuto getPathPlannerAuto() 
    {
        putAutoSwitchesToSmartDashboard();

        PathPlannerAuto pathPlannerAuto = autoMap.get(getAutoSwitchesMask());
        return pathPlannerAuto;
    }

    private static boolean[] getAutoSwitches()
    {
        return new boolean[] {!autoSwitch1.get(), !autoSwitch2.get(), !autoSwitch3.get(), !autoSwitch4.get()};
    }

    private static int getAutoSwitchesMask()
    {
        boolean[] input = getAutoSwitches();
        int mask = convertToMask(input);
        return mask;
    }

    private static int convertToMask(boolean[] input)
    {
        if (input.length != 4)
        {
            return -1;
        }

        int mask = 0;

        if (input[0])
        {
            mask += 1;
        }
        if (input[1])
        {
            mask += 2;
        }
        if (input[2])
        {
            mask += 4;
        }
        if (input[3])
        {
            mask += 8;
        }

        return mask;
    }

    private static void putAutoSwitchesToSmartDashboard()
    {
        boolean[] output = getAutoSwitches();
        SmartDashboard.putBooleanArray("Auto", output);
    }
}
