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

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.time.LocalDateTime;

/** Wrapper for a DataLogManager that includes various robot-agnostic logging features. */
public class HighLevelLogger {
    private static boolean hasStarted = false;
    private static final String commitDataFilename = "commit.txt";
    private static final String branchDataFilename = "branch.txt";

    /**
     * Starts the HighLevelLogger.
     *
     * <p>Upon startup, the logger attempts to read git commit/branch data from the files in the
     * deploy directory of the RoboRio specified in the source for this class (currently
     * "commit.txt" and "branch.txt").
     *
     * <p>These should be updated by the build.gradle of your functional robot code as specified by
     * <a
     * href="https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/deploy-git-data.html">this</a>
     * documentation from WPI.
     */
    public static void startLogging() {
        if (!hasStarted) {
            hasStarted = true;
            DataLogManager.logNetworkTables(false);
            logMessage("Log init");

            try {
                File commitTxt = new File(Filesystem.getDeployDirectory(), commitDataFilename);
                File branchTxt = new File(Filesystem.getDeployDirectory(), branchDataFilename);

                logMessage("Git commit: " + Files.readString(commitTxt.toPath()));
                logMessage("Git branch: " + Files.readString(branchTxt.toPath()));
            } catch (IOException exc) {
                DriverStation.reportWarning("Git branch / commit data could not be read!", false);
                logMessage("Git branch / commit data could not be read!");
            }

        } else {
            DriverStation.reportWarning("HighLevelLogger already started!", false);
            logMessage("HighLevelLogger already started!");
        }

        logMessage("Real time: " + LocalDateTime.now().toString());
        logMessage("Robot is: " + (RobotBase.isReal() ? "real" : "simulated"));
        logMessage(
                "Event: " + (DriverStation.isFMSAttached() ? DriverStation.getEventName() : "N/A"));
        logMessage(
                "Match type: "
                        + (DriverStation.isFMSAttached()
                                ? DriverStation.getMatchType().toString()
                                : "N/A"));
        logMessage(
                "Match number: "
                        + (DriverStation.isFMSAttached() ? DriverStation.getMatchNumber() : "N/A"));
    }

    /**
     * Returns the DataLog populated by this, for use in more-granular and robot-specific logging.
     *
     * @return The DataLog.
     */
    public static DataLog getLog() {
        if (!hasStarted) {
            startLogging();
        }
        return DataLogManager.getLog();
    }

    /**
     * Logs a String message (warning, state transition, startup information, etc.) to the log (not
     * NetworkTables).
     *
     * @param message The message to log
     */
    public static void logMessage(String message) {
        if (!hasStarted) {
            startLogging();
        }
        DataLogManager.log(message);
    }
}
