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
import edu.wpi.first.wpilibj.RobotController;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;

/**
 * {@code IntrinsicLoggable} class for logging of basic RoboRio data (e.g. bus currents and
 * voltages).
 */
public class BaseHardwareLogger implements IntrinsicLoggable {
    private static final BaseHardwareLogger instance = new BaseHardwareLogger();

    private boolean logsConstructed = false;

    private Logger<Boolean> isBrownedOutLogger;

    private Logger<Double> canUtilizationLogger;

    private Logger<Double> inputVoltageLogger;

    private Logger<Double> roboRioCurrentLogger;

    private Logger<Double> logger3_3vCurrent;

    private Logger<Double> logger5vCurrent;

    private Logger<Double> logger6vCurrent;

    private Logger<Double> logger3_3vVoltage;

    private Logger<Double> logger5vVoltage;

    private Logger<Double> logger6vVoltage;

    private BaseHardwareLogger() {}

    /**
     * Returns the singleton instance of BaseHardwareLogger.
     *
     * @return The instance of BaseHardwareLogger.
     */
    public static BaseHardwareLogger getInstance() {
        return instance;
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            LoggerFactory<Double> doubleLogFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            isBrownedOutLogger = new Logger<>("isBrownedOut", subdirName);

            canUtilizationLogger =
                    doubleLogFactory.getLogger(
                            "canUtilitzation_percent",
                            () -> RobotController.getCANStatus().percentBusUtilization);

            inputVoltageLogger =
                    doubleLogFactory.getLogger(
                            "roboRioOnputVoltage_volts", RobotController::getInputVoltage);

            roboRioCurrentLogger =
                    doubleLogFactory.getLogger(
                            "roboRioCurrent_amps", RobotController::getInputCurrent);

            logger3_3vCurrent =
                    doubleLogFactory.getLogger(
                            "roboRio3.3vCurrent_amps", RobotController::getCurrent3V3);

            logger5vCurrent =
                    doubleLogFactory.getLogger(
                            "roboRio5vCurrent_amps", RobotController::getCurrent5V);

            logger6vCurrent =
                    doubleLogFactory.getLogger(
                            "roboRio6vCurrent_amps", RobotController::getCurrent6V);

            logger3_3vVoltage =
                    doubleLogFactory.getLogger(
                            "roboRio3.3vBusVoltage_volts", RobotController::getVoltage3V3);

            logger5vVoltage =
                    doubleLogFactory.getLogger(
                            "roboRio5vBusVoltage_volts", RobotController::getVoltage5V);

            logger6vVoltage =
                    doubleLogFactory.getLogger(
                            "roboRio6vBusVoltage_volts", RobotController::getVoltage6V);

            logsConstructed = true;
        }
    }

    @Override
    public void updateLogs() {
        if (logsConstructed) {
            isBrownedOutLogger.update();
            canUtilizationLogger.update();
            inputVoltageLogger.update();
            roboRioCurrentLogger.update();
            logger3_3vCurrent.update();
            logger5vCurrent.update();
            logger6vCurrent.update();
            logger3_3vVoltage.update();
            logger5vVoltage.update();
            logger6vVoltage.update();
        }
    }
}
