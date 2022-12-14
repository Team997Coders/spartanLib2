/**
Copyright 2022 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with SpartanLib2. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.lib.telemetry;

import static org.junit.Assert.assertEquals;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.lang.reflect.Field;
import java.util.Map;
import org.junit.Test;

/**
 * Tests for the DashboardChooser, using reflection to function without a running NetworkTables
 * server.
 */
public class DashboardChooserTests {
    @Test
    @SuppressWarnings("unchecked")
    public void enumDashboardChooserMatchesSendableChooser()
            throws NoSuchFieldException, IllegalAccessException {
        // Expected SendableChooser
        SendableChooser<OptionEnum> expectedChooser = new SendableChooser<>();

        expectedChooser.addOption(OptionEnum.OPTION_1.getDisplayName(), OptionEnum.OPTION_1);
        expectedChooser.setDefaultOption(OptionEnum.OPTION_2.getDisplayName(), OptionEnum.OPTION_2);
        expectedChooser.addOption(OptionEnum.OPTION_3.getDisplayName(), OptionEnum.OPTION_3);

        // Actual ShuffleboardChooser
        DashboardChooser<OptionEnum> actualChooser =
                DashboardChooser.fromEnum(OptionEnum.class, OptionEnum.OPTION_2, false);

        // reflectively access sendable chooser's display name to value map.
        Field sendableMapField = SendableChooser.class.getDeclaredField("m_map");
        sendableMapField.setAccessible(true);

        // reflectively access ShuffleboardChooser's display name to value map.
        Field shuffleboardMapField = DashboardChooser.class.getDeclaredField("optionMap");
        shuffleboardMapField.setAccessible(true);

        Map<String, OptionEnum> expectedMap =
                (Map<String, OptionEnum>) sendableMapField.get(expectedChooser);
        Map<String, OptionEnum> actualMap =
                (Map<String, OptionEnum>) shuffleboardMapField.get(actualChooser);

        // close choosers
        expectedChooser.close();
        actualChooser.close();

        assertEquals(expectedMap, actualMap);
        assertEquals(expectedChooser.getSelected(), actualChooser.getSelected());
    }

    @Test
    @SuppressWarnings("unchecked")
    public void genericDashboardChooserMatchesSendableChooser()
            throws NoSuchFieldException, IllegalAccessException {
        // Expected SendableChooser
        SendableChooser<String> expectedChooser = new SendableChooser<>();

        expectedChooser.addOption("Option 1", "Value 1");
        expectedChooser.addOption("Option 2", "Value 2");
        expectedChooser.setDefaultOption("Option 3", "Value 3");

        // Actual SendableChooser
        DashboardChooser<String> actualChooser =
                new DashboardChooser<>(
                        Map.of(
                                "Option 1", "Value 1",
                                "Option 2", "Value 2",
                                "Option 3", "Value 3"),
                        "Option 3",
                        false);

        // reflectively access sendable chooser's display name to value map.
        Field sendableMapField = SendableChooser.class.getDeclaredField("m_map");
        sendableMapField.setAccessible(true);

        // reflectively access ShuffleboardChooser's display name to value map.
        Field shuffleboardMapField = DashboardChooser.class.getDeclaredField("optionMap");
        shuffleboardMapField.setAccessible(true);

        Map<String, OptionEnum> expectedMap =
                (Map<String, OptionEnum>) sendableMapField.get(expectedChooser);
        Map<String, OptionEnum> actualMap =
                (Map<String, OptionEnum>) shuffleboardMapField.get(actualChooser);

        // close choosers
        expectedChooser.close();
        actualChooser.close();

        assertEquals(expectedMap, actualMap);
        assertEquals(expectedChooser.getSelected(), actualChooser.getSelected());
    }
}

enum OptionEnum implements DashboardChooser.Option {
    OPTION_1("Option 1"),
    OPTION_2("Option 2"),
    OPTION_3("Option 3");

    private final String displayName;

    OptionEnum(String displayName) {
        this.displayName = displayName;
    }

    @Override
    public String getDisplayName() {
        return displayName;
    }
}
