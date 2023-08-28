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
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

/** Abstract class for "smart" motor controllers, such as SparkMAX and TalonFX. */
public abstract class AbstractSmartMotorController extends AbstractMotorController
        implements StalenessWatchable {

    /**
     * Possible idle modes for a smart motor controller.
     *
     * <p>Coast mode simply cuts all current through the motor, while brake mode shorts together
     * motor leads, imparting signficant stopping force.
     */
    public static enum IdleMode {
        COAST,
        BRAKE;
    }

    private boolean logsConstructed = false;

    private Logger<String> idleModeLogger;

    private Logger<Double> appliedVoltageLogger;

    private Logger<Double> currentLogger;

    private Logger<Double> busVoltageLogger;

    private Logger<Double> motorTemperatureLogger;

    private Logger<Boolean> stalenessWatchdogTriggeredLogger;

    /**
     * Returns the voltage actually applied by the motor controller.
     *
     * <p>This can differ from {@code getSetVoltage()}, as that returns what the user most recently
     * requested.
     *
     * @return Voltage applied by the controller.
     */
    public abstract double getAppliedVoltage();

    /**
     * Sets the idle behavior of this motor controller.
     *
     * @param idleMode Idle mode to now use.
     */
    public abstract void setIdleMode(IdleMode idleMode);

    /**
     * Returns the currently used idle behavior of the motor controller.
     *
     * @return How the motor reacts when given an input of 0.
     */
    public abstract IdleMode getIdleMode();

    /**
     * Returns the main voltage observed by this motor controller.
     *
     * @return Input voltage to this motor controller.
     */
    public abstract double getBusVoltage();

    /**
     * Current, in amps, drawn by this motor controller.
     *
     * @return Combined current draw of motor controller and motor.
     */
    public abstract double getCurrent();

    /**
     * Returns the temperature of the motor, in celsius.
     *
     * @return Motor temperature as measured by the controller.
     */
    public abstract double getMotorTemperature();

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            super.autoGenerateLogs(log, name, subdirName, publishToNT, recordInLog);

            LoggerFactory<Double> factory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            appliedVoltageLogger = factory.getLogger(name + "/appliedVoltage_v");

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
            appliedVoltageLogger.update(getAppliedVoltage());

            currentLogger.update(getCurrent());
            busVoltageLogger.update(getBusVoltage());
            motorTemperatureLogger.update(getMotorTemperature());

            idleModeLogger.update(getIdleMode().toString());
            stalenessWatchdogTriggeredLogger.update(isStale());
        }
    }
}
