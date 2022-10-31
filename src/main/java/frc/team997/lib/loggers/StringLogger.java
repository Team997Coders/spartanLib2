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

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * A class that packages onboard logging and publishing to Shuffleboard / NetworkTables for strings.
 *
 * <p>For instance, this can be used to display and record the state of a system using the {@code
 * .toString()} method of an enum's members.
 *
 * <p>Separate classes are provided for several different data types to enforce type safety for
 * Shuffleboard and external log parsers.
 */
public class StringLogger {
    private final DataLog dataLog;
    private final String tabName;
    private final boolean putToDashboard;
    private final boolean putInLog;
    private final String key;
    private String lastValue;

    private StringLogEntry logEntry;

    /**
     * Constructor for a StringLogger.
     *
     * @param dataLog The log to publish to (most likely from {@code DataLogManager.getLog}). Can be
     *     {@code null} if not intending to publish to log.
     * @param tabName The string identifier of the Shuffleboard tab to publish to. Can be {@code
     *     null} if not intending to publish to dashboard.
     * @param key The string key to associate with this in logging and on the dashboard.
     * @param putToDashboard Whether to publish to dashboard.
     * @param putInLog Whether to publish to log.
     */
    public StringLogger(
            DataLog dataLog, String tabName, String key, Boolean putToDashboard, Boolean putInLog) {
        this.dataLog = dataLog;
        this.tabName = tabName;
        this.key = key;
        this.putToDashboard = putToDashboard;
        this.putInLog = putInLog;
        lastValue = null;

        if (this.putToDashboard && (this.tabName != null)) {
            Shuffleboard.getTab(this.tabName).addString(this.key, () -> lastValue);
        }

        if (this.putInLog && (this.dataLog != null)) {
            logEntry = new StringLogEntry(this.dataLog, this.key);
        }
    }

    /**
     * Constructor for a StringLogger that publishes to both a log and dashboard.
     *
     * @param dataLog The log to publish to (most likely from {@code DataLogManager.getLog})
     * @param tabName The string identifier of the Shuffleboard tab to publish to.
     * @param key The string key to associate with this in logging and on the dashboard.
     */
    public StringLogger(DataLog dataLog, String tabName, String key) {
        this(dataLog, tabName, key, true, true);
    }

    /**
     * Updates log and/or dashboard with a new value.
     *
     * @param value The new string value to update to.
     */
    public void update(String value) {
        if (putInLog && (dataLog != null) && (!value.equals(lastValue))) {
            logEntry.append(value);
        }
        lastValue = value;
    }
}
