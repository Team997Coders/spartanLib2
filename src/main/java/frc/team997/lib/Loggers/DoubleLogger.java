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
package frc.team997.lib.Loggers;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * A class that packages onboard logging and publishing to Shuffleboard / NetworkTables for doubles.
 *
 * <p>For instance, this can display and record the values of an onboard gyroscope.
 *
 * <p>Separate classes are provided for several different data types to enforce type safety for
 * Shuffleboard and external log parsers.
 */
public class DoubleLogger {
    private DataLog m_dataLog;
    private String m_tabName;
    private Boolean m_putToDashboard;
    private Boolean m_putInLog;
    private String m_key;
    private Double m_lastValue;

    private DoubleLogEntry m_logEntry;

    /**
     * Constructor for a DoubleLogger.
     *
     * @param dataLog The log to publish to (most likely from {@code DataLogManager.getLog}). Can be
     *     {@code null} if not intending to publish to log.
     * @param tabName The string identifier of the Shuffleboard tab to publish to. Can be {@code
     *     null} if not intending to publish to dashboard.
     * @param key The string key to associate with this in logging and on the dashboard.
     * @param putToDashboard Whether to publish to dashboard.
     * @param putInLog Whether to publish to log.
     */
    public DoubleLogger(
            DataLog dataLog, String tabName, String key, Boolean putToDashboard, Boolean putInLog) {
        m_dataLog = dataLog;
        m_tabName = tabName;
        m_key = key;
        m_putToDashboard = putToDashboard;
        m_putInLog = putInLog;
        m_lastValue = null;

        if (m_putToDashboard && (m_tabName != null)) {
            Shuffleboard.getTab(m_tabName)
                    .addNumber(
                            m_key,
                            () -> {
                                return m_lastValue;
                            });
        }

        if (m_putInLog && (m_dataLog != null)) {
            m_logEntry = new DoubleLogEntry(m_dataLog, m_key);
        }
    }

    /**
     * Constructor for a DoubleLogger that publishes to both a log and dashboard.
     *
     * @param dataLog The log to publish to (most likely from {@code DataLogManager.getLog})
     * @param tabName The string identifier of the Shuffleboard tab to publish to.
     * @param key The string key to associate with this in logging and on the dashboard.
     */
    public DoubleLogger(DataLog dataLog, String tabName, String key) {
        this(dataLog, tabName, key, true, true);
    }

    /**
     * Updates log and/or dashboard with a new value.
     *
     * @param value The new double value to update to.
     */
    public void update(Double value) {
        if (m_putInLog && (m_dataLog != null) && (value != m_lastValue)) {
            m_logEntry.append(value);
        }
        m_lastValue = value;
    }
}
