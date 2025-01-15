package frc.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShuffleUtils {
    public static GenericEntry getEntryByName(ShuffleboardTab tab, String name) {
        for (var item : tab.getComponents())
            if (item.getTitle() == name)
                return ((SimpleWidget)item).getEntry();

        return null;
    }
}
