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
package frc.team997.lib.loggers;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * A class that packages onboard logging and publishing to Shuffleboard / NetworkTables for
 * booleans.
 *
 * <p>For instance, this can display and record whether a digital limit switch is pressed.
 *
 * <p>Separate classes are provided for several different data types to enforce type safety for
 * Shuffleboard and external log parsers.
 */
public class BooleanLogger {
    private final DataLog dataLog;
    private final String tabName;
    private final Boolean putToDashboard;
    private final Boolean putInLog;
    private final String key;
    private Boolean lastValue;

    private BooleanLogEntry logEntry;

    /**
     * Constructor for a BooleanLogger.
     *
     * @param dataLog The log to publish to (most likely from {@code DataLogManager.getLog}). Can be
     *     {@code null} if not intending to publish to log.
     * @param tabName The string identifier of the Shuffleboard tab to publish to. Can be {@code
     *     null} if not intending to publish to dashboard.
     * @param key The string key to associate with this in logging and on the dashboard.
     * @param putToDashboard Whether to publish to dashboard.
     * @param putInLog Whether to publish to log.
     */
    public BooleanLogger(
            DataLog dataLog, String tabName, String key, Boolean putToDashboard, Boolean putInLog) {
        this.dataLog = dataLog;
        this.tabName = tabName;
        this.key = key;
        this.putToDashboard = putToDashboard;
        this.putInLog = putInLog;
        lastValue = null;

        if (this.putToDashboard && (this.tabName != null)) {
            Shuffleboard.getTab(this.tabName)
                    .addBoolean(
                            this.key,
                            () -> lastValue);
        }

        if (this.putInLog && (this.dataLog != null)) {
            logEntry = new BooleanLogEntry(this.dataLog, this.key);
        }
    }

    /**
     * Constructor for a BooleanLogger that publishes to both a log and dashboard.
     *
     * @param dataLog The log to publish to (most likely from {@code DataLogManager.getLog})
     * @param tabName The string identifier of the Shuffleboard tab to publish to.
     * @param key The string key to associate with this in logging and on the dashboard.
     */
    public BooleanLogger(DataLog dataLog, String tabName, String key) {
        this(dataLog, tabName, key, true, true);
    }

    /**
     * Updates log and/or dashboard with a new value.
     *
     * @param value The new boolean value to update to.
     */
    public void update(Boolean value) {
        if (putInLog && (dataLog != null) && (value != lastValue)) {
            logEntry.append(value);
        }
        lastValue = value;
    }
}
