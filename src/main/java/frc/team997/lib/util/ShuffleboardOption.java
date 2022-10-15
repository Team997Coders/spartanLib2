/*
 * Copyright 2022 FRC Team 997
 *
 * This program is free software:
 * you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with SpartanLib2.
 * If not, see <https://www.gnu.org/licenses/>.
 */

package frc.team997.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.security.InvalidParameterException;
import java.util.HashMap;
import java.util.Map;

public class ShuffleboardOption<T> {
    private final SendableChooser<T> sendableChooser;
    public SendableChooser<T> getSendableChooser() {
        return sendableChooser;
    }
    public interface ShuffleboardEntry {
        String getDisplayName();
    }
    public static <T> ShuffleboardOption<T> fromEnum(Class<? extends ShuffleboardEntry> enumClass, ShuffleboardEntry defaultOption) {
        Map<String, T> options = new HashMap<>();
        for (ShuffleboardEntry entry : enumClass.getEnumConstants()) {
            options.put(entry.getDisplayName(), (T) entry);
        }
        options.put(defaultOption.getDisplayName(), (T) defaultOption);
        return new ShuffleboardOption<T>(options, (T) defaultOption);
    }
    public ShuffleboardOption(Map<String, T> options, T defaultOption) {
        // I watched this break the drivetrain - every shuffleboard option should
        // have a default, but if not the default should be explicitly passed as null.
        if (defaultOption != null && !options.containsValue(defaultOption)) {
            throw new InvalidParameterException("defaultOption must be either null or a valid option value");
        }
        sendableChooser = new SendableChooser<>();
        for (Map.Entry<String, T> entry : options.entrySet()) {
            if (entry.getValue().equals(defaultOption)) {
                sendableChooser.setDefaultOption(entry.getKey(), entry.getValue());
            } else {
                sendableChooser.addOption(entry.getKey(), entry.getValue());
            }
        }
    }
    public T getSelected() {
        return sendableChooser.getSelected();
    }
    public void close() {
        sendableChooser.close();
    }
}
