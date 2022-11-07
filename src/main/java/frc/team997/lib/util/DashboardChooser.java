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
import edu.wpi.first.wpilibj.DataLogManager;
import frc.team997.lib.telemetry.Logger;
import java.security.InvalidParameterException;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A NTSendable which replaces the SendableChooser in favor of a listener-based api. Used for
 * constructing option widgets in the dashboard.
 *
 * @param <T> The data type of the option.
 */
public class DashboardChooser<T> implements NTSendable, AutoCloseable {
    private static final AtomicInteger numInstances = new AtomicInteger();

    private final int channel;
    private final Map<String, T> optionMap;
    private final String defaultOption;
    private String selectedOption;

    private final Set<ValueUpdater<T>> listeners = new LinkedHashSet<>();
    private final Set<NetworkTableEntry> activeEntries = new LinkedHashSet<>();
    private final ReentrantLock mutex = new ReentrantLock();

    private Logger<T> logger;
    private final boolean logChanges;

    public interface Option {
        String getDisplayName();
    }

    public interface ValueUpdater<T> {
        void onOptionSelected(T oldOption, T newOption);
    }

    /**
     * Constructs a DashboardChooser from an enum implementing {@link Option}.
     *
     * @param enumClass The {@code <enum>.class}
     * @param defaultOption The default enum option
     * @param logChanges Whether to log changes in the chosen value to an on-robot log.
     * @return A new DashboardChooser object.
     * @param <T> The enum's type.
     */
    @SuppressWarnings("unchecked")
    public static <T> DashboardChooser<T> fromEnum(
            Class<? extends Option> enumClass, Option defaultOption, boolean logChanges) {
        Map<String, T> options = new HashMap<>();
        for (Option entry : enumClass.getEnumConstants()) {
            options.put(entry.getDisplayName(), (T) entry);
        }
        options.put(defaultOption.getDisplayName(), (T) defaultOption);
        return new DashboardChooser<T>(options, defaultOption.getDisplayName(), logChanges);
    }

    /**
     * Constructs a DashboardChooser from an enum implementing {@link Option}, saving changes to an
     * on-robot log.
     *
     * @param enumClass The {@code <enum>.class}
     * @param defaultOption The default enum option
     * @return A new DashboardChooser object.
     * @param <T> The enum's type.
     */
    public static <T> DashboardChooser<T> fromEnum(
            Class<? extends Option> enumClass, Option defaultOption) {
        return fromEnum(enumClass, defaultOption, true);
    }

    /**
     * Constructs a DashboardChooser from a {@code Map<String, T>}.
     *
     * @param options The option list, as a Map<title, value>.
     * @param defaultOption The title of the default option.
     * @param logChanges Whether to log changes in the chosen value to an on-robot log.
     * @throws InvalidParameterException If the default option is neither {@code null} or part of
     *     the options.
     */
    public DashboardChooser(Map<String, T> options, String defaultOption, boolean logChanges)
            throws InvalidParameterException {
        if (defaultOption != null && !options.containsKey(defaultOption)) {
            throw new InvalidParameterException(
                    "defaultOption must be either null or a valid option value");
        }
        // set the channel to the nex available channel (each instance gets its own channel)
        channel = numInstances.getAndIncrement();

        if (logChanges) {
            logger =
                    new Logger<T>(
                            DataLogManager.getLog(),
                            "DashboardChooser" + channel,
                            "DashboardChooser",
                            false,
                            true);
        }

        this.logChanges = logChanges;

        // register the chooser with NT
        SendableRegistry.add(this, "DashboardChooser", channel);

        optionMap = options;
        this.defaultOption = defaultOption;
    }

    /**
     * Constructs a DashboardChooser from a {@code Map<String, T>}, recording changes to the chosen
     * value in an on-robot log.
     *
     * @param options The option list, as a Map<title, value>.
     * @param defaultOption The title of the default option.
     * @throws InvalidParameterException If the default option is neither {@code null} or part of
     *     the options.
     */
    public DashboardChooser(Map<String, T> options, String defaultOption)
            throws InvalidParameterException {
        this(options, defaultOption, true);
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

        if (logChanges) logger.update(newOption);

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
