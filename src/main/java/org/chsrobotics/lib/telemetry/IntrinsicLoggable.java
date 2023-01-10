/**
Copyright 2023 FRC Team 997

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

import edu.wpi.first.util.datalog.DataLog;

/** Interface for classes which can auto-generate {@link Logger}s for internal fields. */
public interface IntrinsicLoggable {
    /**
     * Auto-generates {@link Logger}s for important internal values.
     *
     * @param log The DataLog to log values inside of, most likely from HighLevelLogger.getLog() or
     *     whatever log is being used program-wide.
     * @param name The name to associate with this object's logged data.
     * @param subdirName The string name of the existing or new NetworkTables sub-table to write to.
     * @param publishToNT Whether this should push logged values to NetworkTables.
     * @param recordInLog Whether this should store logged values in an on-robot log file.
     */
    void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog);

    /**
     * Auto-generates {@link Logger}s for important and notable internal values, using the default
     * DataLog, and both publishing to NT and logging on-robot.
     *
     * @param name The name to associate with this object's logged data.
     * @param subdirName The string name of the existing or new NetworkTables sub-table to write to.
     */
    default void autoGenerateLogs(String name, String subdirName) {
        autoGenerateLogs(HighLevelLogger.getInstance().getLog(), name, subdirName, true, true);
    }

    /** Updates the internally generated Loggers. */
    void updateLogs();
}
