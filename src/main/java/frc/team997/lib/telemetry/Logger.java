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
package frc.team997.lib.telemetry;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * The Logger class conveniently packages publishing telemetry data to a off-robot dashboard
 * (Shuffleboard) and saving that telemetry to an on-robot log file.
 *
 * <p>Parameterized to the type of data to publish/log.
 *
 * <p>If the data isn't numerical (Double, Int, Long) or Boolean/String, this will instead publish
 * and log the result of its {@code .toString()} method.
 */
public class Logger<T> {
    private String key;
    private String tabName;
    private boolean publishToDashboard;
    private boolean recordInLog;

    private BooleanLogEntry boolLogEntry;
    private DoubleLogEntry doubleLogEntry;
    private StringLogEntry stringLogEntry;

    private boolean entryAlreadyExists;

    private T currentDashboardValue;

    /**
     * Constructs a Logger using a provided DataLog, with the option to not publish to the dashboard
     * or record in log.
     *
     * @param log The DataLog to log values inside of, most likely from {@code
     *     HighLevelLogger.getLog()} or whatever log is being used program-wide.
     * @param key A string identifier to associate with and describe the data on the dashboard. In
     *     the log, the string identifier will be "{@code [tabName]_[key]}".
     * @param tabName The string name of the existing or new Shuffleboard tab this will publish to.
     *     It is recommended to create a constant holding this, to avoid problems with
     *     capitalization/whitespace.
     * @param publishToDashboard Whether this should push logged values to NetworkTables and an
     *     off-robot dashboard (Shuffleboard).
     * @param recordInLog Whether this should store logged values in an on-robot log file.
     */
    public Logger(
            DataLog log,
            String key,
            String tabName,
            boolean publishToDashboard,
            boolean recordInLog) {
        this.key = key;
        this.tabName = tabName;
        this.publishToDashboard = publishToDashboard;
        this.recordInLog = recordInLog;

        entryAlreadyExists = false;

        String logIdentifier = tabName + "_" + key;

        // clunky but negligible performance impact and won't clutter log with unnessecary artifacts
        boolLogEntry = new BooleanLogEntry(log, logIdentifier);
        doubleLogEntry = new DoubleLogEntry(log, logIdentifier);
        stringLogEntry = new StringLogEntry(log, logIdentifier);
    }

    /**
     * Constructs a Logger using {@code HighLevelLogger.getLog()}, publishing to dashboard and
     * logging.
     *
     * @param key A string identifier to associate with and describe the data on the dashboard. In
     *     the log, the string identifier will be "{@code [tabName]_[key]}".
     * @param tabName The string name of the existing or new Shuffleboard tab this will publish to.
     *     It is recommended to create a constant holding this, to avoid problems with
     *     capitalization/whitespace.
     */
    public Logger(String key, String tabName) {
        this(DataLogManager.getLog(), key, tabName, true, true);
    }

    /**
     * Returns whether this is publishing values to the dashboard.
     *
     * @return True if new values sent to {@code update()} will be pushed to the dashboard.
     */
    public boolean isPublishingToDashboard() {
        return publishToDashboard;
    }

    /** New values sent to {@code update()} will be pushed to the dashboard. */
    public void startPublishingToDashboard() {
        publishToDashboard = true;
    }

    /**
     * New values sent to {@code update()} will not be pushed to the dashboard.
     *
     * <p>Note that this field will remain on Shuffleboard, and that the GUI will not properly
     * gray-out to indicate that the value is stale.
     */
    public void stopPublishingToDashboard() {
        publishToDashboard = false;
    }

    /**
     * Returns whether this is recording values to the on-robot log.
     *
     * @return True if new values sent to {@code update()} will be recorded to the log.
     */
    public boolean isRecordingToLog() {
        return recordInLog;
    }

    /** New values sent to {@code update()} will be recorded to the log. */
    public void startRecordingToLog() {
        recordInLog = true;
    }

    /** New values sent to {@code update()} will not be recorded to the log. */
    public void stopRecordingToLog() {
        recordInLog = false;
    }

    /**
     * Feeds a new value to the Logger, which may be sent to an on-robot log, an off-robot log,
     * both, or neither, depending on how this class is configured.
     *
     * @param value An instance of T (whatever this class was parameterized with).
     */
    public void update(T value) {
        if (publishToDashboard) currentDashboardValue = value;
        if (value instanceof Double || value instanceof Integer || value instanceof Long) {
            if (recordInLog) doubleLogEntry.append((double) value);
            if (publishToDashboard && !entryAlreadyExists) {
                entryAlreadyExists = true;
                Shuffleboard.getTab(tabName)
                        .addNumber(
                                key,
                                () -> {
                                    return (double) currentDashboardValue;
                                });
            }
        } else if (value instanceof Boolean) {
            if (recordInLog) boolLogEntry.append((boolean) value);
            if (publishToDashboard && !entryAlreadyExists) {
                entryAlreadyExists = true;
                Shuffleboard.getTab(tabName)
                        .addBoolean(
                                key,
                                () -> {
                                    return (boolean) currentDashboardValue;
                                });
            }
        } else if (value instanceof String) {
            if (recordInLog) stringLogEntry.append((String) value);
            if (publishToDashboard && !entryAlreadyExists) {
                entryAlreadyExists = true;
                Shuffleboard.getTab(tabName)
                        .addString(
                                key,
                                () -> {
                                    return (String) currentDashboardValue;
                                });
            }
        } else {
            if (recordInLog) stringLogEntry.append(value.toString());
            if (publishToDashboard && !entryAlreadyExists) {
                entryAlreadyExists = true;
                Shuffleboard.getTab(tabName)
                        .addString(
                                key,
                                () -> {
                                    return currentDashboardValue.toString();
                                });
            }
        }
    }
}
