package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

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

    private AutoSwitchHelpers() {

    }

    public static void put(boolean[] input, Command command) {

        if (input.length != 4)
        {
            System.err.println("[AutoSwitchHelpers.put]: input must be length 4");
            return;
        }

        int mask = convertToMask(input);

        autoMap.put(mask, command);
    }

    public static void put(int mask, Command command)
    {
        if (mask < 0 || mask >= 16)
        {
            System.err.println("[AutoSwitchHelpers.put]: mask must be between 0 and 15");
            return;
        }

        autoMap.put(mask, command);
    }

    public static Command getCommand() {
        
        return autoMap.get(getSwitchMask());
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
