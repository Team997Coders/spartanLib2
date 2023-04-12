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
package org.chsrobotics.lib.hardware.motorController;

import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public abstract class AbstractSmartMotorController extends AbstractMotorController
        implements StalenessWatchable {
    public static enum IdleMode {
        COAST,
        BRAKE;

        public com.revrobotics.CANSparkMax.IdleMode asRev() {
            if (this == IdleMode.BRAKE) return com.revrobotics.CANSparkMax.IdleMode.kBrake;
            else return com.revrobotics.CANSparkMax.IdleMode.kCoast;
        }
    }

    private boolean logsConstructed = false;

    private Logger<String> idleModeLogger;

    private Logger<Double> currentLogger;

    private Logger<Double> busVoltageLogger;

    private Logger<Double> motorTemperatureLogger;

    private Logger<Boolean> stalenessWatchdogTriggeredLogger;

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract IdleMode getIdleMode();

    public abstract double getBusVoltage();

    public abstract double getCurrent();

    public abstract double getMotorTemperature();

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            super.autoGenerateLogs(log, name, subdirName, publishToNT, recordInLog);

            LoggerFactory<Double> factory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            currentLogger = factory.getLogger(name + "/current_A");
            busVoltageLogger = factory.getLogger(name + "/busVoltage_v");
            motorTemperatureLogger = factory.getLogger(name + "/motorTemperature_c");

            idleModeLogger =
                    new Logger<>(log, name + "/idleMode", subdirName, publishToNT, recordInLog);

            stalenessWatchdogTriggeredLogger =
                    new Logger<>(log, name + "/isStale", subdirName, publishToNT, recordInLog);

            logsConstructed = true;

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            currentLogger.update(getCurrent());
            busVoltageLogger.update(getBusVoltage());
            motorTemperatureLogger.update(getMotorTemperature());

            idleModeLogger.update(getIdleMode().toString());
            stalenessWatchdogTriggeredLogger.update(isStale());
        }
    }
}
