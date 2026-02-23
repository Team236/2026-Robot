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

    private static HashMap<Integer, Command> autoMap = new HashMap<>();

    static {
        autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
        autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
        autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
        autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);
    }

    private AutoSwitchHelpers() {}

    public static void put(boolean s1, boolean s2, boolean s3, boolean s4, Command autoCommand) {

        boolean[] input = {s1, s2, s3, s4};

        if (input.length != 4)
        {
            System.err.println("[AutoSwitchHelpers.put]: input must be length 4");
            return;
        }

        int mask = convertToMask(input);

        autoMap.put(mask, autoCommand);
    }

    public static void put(int mask, Command autoCommand)
    {
        if (mask < 0 || mask >= 16)
        {
            System.err.println("[AutoSwitchHelpers.put]: mask must be between 0 and 15");
            return;
        }

        autoMap.put(mask, autoCommand);
    }

    public static Command getAutoCommand() {
        Command autoCommandToRun = autoMap.get(getSwitchMask());
        
        if (autoCommandToRun == null) { 
            // SmartDashboard.putString("Auto", "No command set");
            return Commands.none(); 
        }
        
        return autoCommandToRun;
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

    private static int getSwitchMask() {

        boolean[] input = new boolean[] {!autoSwitch1.get(), !autoSwitch2.get(), !autoSwitch3.get(), !autoSwitch4.get()};

        int mask = convertToMask(input);

        return mask;
    }
}
