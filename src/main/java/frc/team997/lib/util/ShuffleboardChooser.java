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
package frc.team997.lib.util;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.security.InvalidParameterException;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A NTSendable which abstracts the SendableChooser api in favor of a listener based api
 * Used for constructing option widgets in Shuffleboard
 *
 * @param <T> The option data type
 */
public class ShuffleboardChooser<T> implements NTSendable, AutoCloseable {
    private static final AtomicInteger numInstances = new AtomicInteger();

    private final int channel;
    private final Map<String, T> optionMap;
    private final String defaultOption;
    private String selectedOption;

    private final Set<ValueUpdater<T>> listeners = new LinkedHashSet<>();
    private final Set<NetworkTableEntry> activeEntries = new LinkedHashSet<>();
    private final ReentrantLock mutex = new ReentrantLock();

    public interface ShuffleboardOption {
        String getDisplayName();
    }

    public interface ValueUpdater<T> {
        void onOptionSelected(T oldOption, T newOption);
    }

    /**
     * Constructs a ShuffleboardChooser from an enum implementing {@link ShuffleboardOption}
     *
     * @param enumClass The {@code <enum>.class}
     * @param defaultOption The default enum option
     * @return A new ShuffleboardChooser object
     * @param <T> The enum's type
     */
    @SuppressWarnings("unchecked")
    public static <T> ShuffleboardChooser<T> fromEnum(
            Class<? extends ShuffleboardOption> enumClass, ShuffleboardOption defaultOption) {
        Map<String, T> options = new HashMap<>();
        for (ShuffleboardOption entry : enumClass.getEnumConstants()) {
            options.put(entry.getDisplayName(), (T) entry);
        }
        options.put(defaultOption.getDisplayName(), (T) defaultOption);
        return new ShuffleboardChooser<T>(options, defaultOption.getDisplayName());
    }

    /**
     * Constructs a ShuffleboardChooser from a {@code Map<String, T>}
     *
     * @param options The option list, as a Map<title, value>
     * @param defaultOption The title of the default option
     */
    public ShuffleboardChooser(Map<String, T> options, String defaultOption) {
        // I watched this break the drivetrain - every shuffleboard option should
        // have a default, but if not the default should be explicitly passed as null.
        if (defaultOption != null && !options.containsKey(defaultOption)) {
            throw new InvalidParameterException(
                    "defaultOption must be either null or a valid option value");
        }
        // set the channel to the nex available channel (each instance gets it's own channel)
        channel = numInstances.getAndIncrement();
        // register the chooser with shuffleboard
        SendableRegistry.add(this, "ShuffleboardChooser", channel);

        optionMap = options;
        this.defaultOption = defaultOption;
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Gets the currently selected option's value
     *
     * @return T value
     */
    public T getSelected() {
        mutex.lock();
        try {
            if (selectedOption == null) {
                return optionMap.get(defaultOption);
            } else {
                return optionMap.get(selectedOption);
            }
        } finally {
            mutex.unlock();
        }
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        // sets the channel
        builder.getEntry(".instance").setDouble(channel);
        // set the default option
        builder.addStringProperty("default", () -> defaultOption, null);
        // populate the options
        builder.addStringArrayProperty(
                "options", () -> optionMap.keySet().toArray(new String[0]), null);
        // define the function for shuffleboard to get the default option
        builder.addStringProperty(
                "active",
                () -> {
                    mutex.lock();
                    try {
                        return selectedOption == null ? defaultOption : selectedOption;
                    } finally {
                        mutex.unlock();
                    }
                },
                null);
        mutex.lock();
        try {
            activeEntries.add(builder.getEntry("active"));
        } finally {
            mutex.unlock();
        }
        // defines the function called with the value has been updated in shuffleboard
        builder.addStringProperty("selected", null, this::onOptionUpdate);
    }

    private void onOptionUpdate(String option) {
        // get value before change
        T oldOption = getSelected();
        mutex.lock();
        try {
            selectedOption = option;
            for (NetworkTableEntry entry : activeEntries) {
                entry.setString(option);
            }
        } finally {
            mutex.unlock();
        }
        // get value after
        T newOption = getSelected();

        // call listeners
        for (ValueUpdater<T> listener : listeners) {
            listener.onOptionSelected(oldOption, newOption);
        }
    }

    /**
     * Register a listener to be called when the option is updated
     *
     * @param updater The lambda to be called on update
     */
    public void registerListener(ValueUpdater<T> updater) {
        listeners.add(updater);
    }

    /**
     * Unregister a listener when no longer needed
     *
     * @param updater the lambda to remove
     */
    public void unregisterListener(ValueUpdater<T> updater) {
        listeners.remove(updater);
    }
}
