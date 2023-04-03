/**
Copyright 2022-2023 FRC Team 997

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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

/**
 * Convenience wrapper class for telemetry/ logging with built-in logging for robot-agnostic data
 * like environment metadata, RoboRio status, and scheduled commands.
 *
 * <p>For the internally included loggers of system status to function, the method {@code
 * logPeriodic()} needs to be called once per robot loop cycle.
 */
public class HighLevelLogger implements IntrinsicLoggable {
    private static HighLevelLogger instance = new HighLevelLogger();

    private boolean hasStarted = false;
    private final String commitDataFilename = "commit.txt";
    private final String branchDataFilename = "branch.txt";

    private final HashMap<Command, Timer> commandTimeMap = new HashMap<>();
    private int brownoutCounter = 0;

    private Logger<String[]> scheduledCommandsLogger;

    private Logger<Boolean> isBrownedOutLogger;

    private Logger<Double> canUtilizationLogger;

    private Logger<Double> batteryVoltageLogger;

    private Logger<Double> logger3_3vCurrent;

    private Logger<Double> logger5vCurrent;

    private Logger<Integer> brownoutCountLogger;

    private Logger<Double> loopTimeLogger;

    private boolean loggersConstructed = false;

    private final NetworkTable sendables = NetworkTableInstance.getDefault().getTable("sendables");

    private final Map<String, Sendable> tablesToData = new HashMap<>();

    private HighLevelLogger() {}

    public static HighLevelLogger getInstance() {
        return instance;
    }

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
    public void startLogging() {
        if (!hasStarted) {
            CommandScheduler.getInstance().onCommandInitialize(getInstance()::logCommandInit);
            CommandScheduler.getInstance().onCommandFinish(getInstance()::logCommandFinished);
            CommandScheduler.getInstance().onCommandInterrupt(getInstance()::logCommandInterrupted);

            hasStarted = true;
            DataLogManager.logNetworkTables(false);
            logMessage("Log init");

            logMessage("Real time: " + LocalDateTime.now().toString());
            logMessage("Robot is: " + (RobotBase.isReal() ? "real" : "simulated"));
            logMessage(
                    "Event: "
                            + (DriverStation.isFMSAttached()
                                    ? DriverStation.getEventName()
                                    : "N/A"));
            logMessage(
                    "Match type: "
                            + (DriverStation.isFMSAttached()
                                    ? DriverStation.getMatchType().toString()
                                    : "N/A"));
            logMessage(
                    "Match number: "
                            + (DriverStation.isFMSAttached()
                                    ? DriverStation.getMatchNumber()
                                    : "N/A"));
            try {
                File commitTxt = new File(Filesystem.getDeployDirectory(), commitDataFilename);
                File branchTxt = new File(Filesystem.getDeployDirectory(), branchDataFilename);

                logMessage("Git commit: " + Files.readString(commitTxt.toPath()));
                logMessage("Git branch: " + Files.readString(branchTxt.toPath()));
            } catch (IOException exc) {
                logMessage("Git branch / commit data could not be read!");
            }
        }
    }

    /**
     * Returns the DataLog populated by this, for use in more-granular and robot-specific logging.
     *
     * @return The DataLog.
     */
    public DataLog getLog() {
        if (!hasStarted) {
            startLogging();
        }
        return DataLogManager.getLog();
    }

    /**
     * Logs a String message (warning, state transition, startup information, etc.), and prints it
     * to the standard output (DriverStation console).
     *
     * @param message The message to log.
     */
    public void logMessage(String message) {
        if (!hasStarted) {
            startLogging();
        }
        DataLogManager.log(message);
    }

    /**
     * Writes a warning, using the provided string, to the DriverStation console, and writes it to
     * the log.
     *
     * @param message The String message to associate with the warning.
     */
    public void logWarning(String message) {
        logMessage("WARNING " + message);
        DriverStation.reportWarning(message, false);
    }

    /**
     * Writes an error, using the provided string, to the DriverStation console, and writes it to
     * the log.
     *
     * @param message The String message to associate with the error.
     */
    public void logError(String message) {
        logMessage("ERROR " + message);
        DriverStation.reportError(message, false);
    }

    /**
     * Publishes a Sendable object to NetworkTables.
     *
     * @param key String key to associate with the object.
     * @param data Sendable object.
     */
    public synchronized void publishSendable(String key, Sendable data) {
        Sendable sddata = tablesToData.get(key);
        if (sddata == null || sddata != data) {
            tablesToData.put(key, data);
            NetworkTable dataTable = sendables.getSubTable(key);
            SendableBuilderImpl builder = new SendableBuilderImpl();
            builder.setTable(dataTable);
            SendableRegistry.publish(data, builder);
            builder.startListeners();
            dataTable.getEntry(".name").setString(key);
        }
    }

    @Override
    /** {@inheritDoc} */
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!loggersConstructed) {
            LoggerFactory<Double> doubleLogFactory =
                    new LoggerFactory<>(log, "system", publishToNT, recordInLog);

            scheduledCommandsLogger = new Logger<>("scheduledCommands", "commandScheduler");

            isBrownedOutLogger = new Logger<>("isBrownedOut", "system");

            canUtilizationLogger = doubleLogFactory.getLogger("canUtilitzation_percent");

            batteryVoltageLogger = doubleLogFactory.getLogger("batteryVoltage_volts");

            logger3_3vCurrent = doubleLogFactory.getLogger("3.3vCurrent_amps");

            logger5vCurrent = doubleLogFactory.getLogger("5vCurrent_amps");

            brownoutCountLogger = new Logger<>("brownoutCount", "system");

            loopTimeLogger = doubleLogFactory.getLogger("loopTime_seconds");

            loggersConstructed = true;

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs(double dtSeconds) {
        if (!loggersConstructed) {
            if (RobotController.getBatteryVoltage() < RobotController.getBrownoutVoltage()) {
                brownoutCounter++;
            }
        } else {
            ArrayList<String> commands = new ArrayList<>();

            for (Command command : commandTimeMap.keySet()) {
                commands.add(command.getName());
            }

            scheduledCommandsLogger.update(commands.toArray(new String[] {}));

            if (RobotController.getBatteryVoltage() < RobotController.getBrownoutVoltage()) {
                brownoutCounter++;
                isBrownedOutLogger.update(true);
            } else {
                isBrownedOutLogger.update(false);
            }

            brownoutCountLogger.update(brownoutCounter);

            canUtilizationLogger.update(RobotController.getCANStatus().percentBusUtilization);

            batteryVoltageLogger.update(RobotController.getBatteryVoltage());

            logger3_3vCurrent.update(RobotController.getCurrent3V3());
            logger5vCurrent.update(RobotController.getCurrent5V());

            loopTimeLogger.update(dtSeconds);
        }
    }

    private void logCommandInit(Command command) {
        logMessage("Command initialized: " + command.getName());

        Timer timer = new Timer();
        timer.reset();
        timer.start();

        commandTimeMap.put(command, timer);
    }

    private void logCommandFinished(Command command) {
        logMessage(
                "Command finished after "
                        + commandTimeMap.get(command).get()
                        + " seconds: "
                        + command.getName());

        commandTimeMap.remove(command);
    }

    private void logCommandInterrupted(Command command) {
        logMessage(
                "Command interrupted after "
                        + commandTimeMap.get(command).get()
                        + " seconds: "
                        + command.getName());

        commandTimeMap.remove(command);
    }
}
