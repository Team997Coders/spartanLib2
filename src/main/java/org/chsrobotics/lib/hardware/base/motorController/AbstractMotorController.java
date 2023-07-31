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
package org.chsrobotics.lib.hardware.base.motorController;

import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

/** Abstract class for all motor controllers. */
public abstract class AbstractMotorController implements IntrinsicLoggable {
    private Logger<Double> setVoltageLogger;

    private boolean logsConstructed = false;

    /**
     * Desired voltage output to this motor controller.
     *
     * <p>For devices where direct voltage control is not exposed, this instead divides by the
     * current bus voltage (or 12) to determine what duty cycle to command.
     *
     * @param volts
     */
    public abstract void setVoltage(double volts);

    /**
     * Returns the most recent set voltage output.
     *
     * @return The most recently commanded voltage. May not match actual motor controller output
     *     voltage, especially if a remote control loop is running.
     */
    public abstract double getSetVoltage();

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            setVoltageLogger =
                    new Logger<>(log, name + "/setVoltage_V", subdirName, publishToNT, recordInLog);

            logsConstructed = true;

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            setVoltageLogger.update(getSetVoltage());
        }
    }
}
