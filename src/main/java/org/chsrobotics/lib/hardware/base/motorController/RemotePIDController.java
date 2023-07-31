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
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

/**
 * Abstract class for PID controllers running onboard motor controllers.
 *
 * @see org.chsrobotics.lib.controllers.feedback.PID RoboRIO implementation of a PID control loop
 *     (containing explanation on the process and tuning).
 */
public abstract class RemotePIDController implements IntrinsicLoggable {

    private Logger<RemoteControlType> controlTypeLogger;

    private Logger<Double> appliedFeedforwardLogger;

    private Logger<Double> kPLogger;
    private Logger<Double> kILogger;
    private Logger<Double> kDLogger;

    private Logger<Double> maxULogger;
    private Logger<Double> maxIntegralULogger;

    private boolean logsConstructed = false;

    /**
     * Enum of possible closed-loop control modes. Coorespoinding units, as always, should be in
     * radians, seconds, amps.
     */
    public static enum RemoteControlType {
        POSITION,
        CONTINUOUS_ANGLE,
        VELOCITY,
        CURRENT,
    }

    /**
     * Returns the current closed-loop control type.
     *
     * @return The active control type.
     */
    public abstract RemoteControlType getControlType();

    /**
     * Sets feedforward to be added to the result of the remote PID controller.
     *
     * <p>This parameter is not sent to the motor controller until {@code setSetpoint()} is called.
     *
     * @param feedforwardVolts Feedforward value in volts.
     */
    public abstract void setFeedforwardVolts(double feedforwardVolts);

    /**
     * Returns the feedforward value currently being used on the motor controller.
     *
     * @return The currently applied feedforward value.
     */
    public abstract double getFeedforwardVolts();

    /**
     * Returns the proportional gain currently being used on the motor controller.
     *
     * @return kP
     */
    public abstract double getkP();

    /**
     * Sets kP to be used on the motor controller.
     *
     * <p>This parameter is not sent to the motor controller until {@code setSetpoint()} is called.
     *
     * @param kP kP
     */
    public abstract void setkP(double kP);

    /**
     * Returns the integral gain currently being used on the motor controller.
     *
     * @return kI
     */
    public abstract double getkI();

    /**
     * Sets kI to be used on the motor controller.
     *
     * <p>This parameter is not sent to the motor controller until {@code setSetpoint()} is called.
     *
     * @param kI kI
     */
    public abstract void setkI(double kI);

    /**
     * Returns the derivative gain currently being used on the motor controller.
     *
     * @return kD
     */
    public abstract double getkD();

    /**
     * Sets kD to be used on the motor controller.
     *
     * <p>This parameter is not sent to the motor controller until {@code setSetpoint()} is called.
     *
     * @param kD kD
     */
    public abstract void setkD(double kD);

    /**
     * Sets the maximum control effort of the motor in either direction, in volts.
     *
     * <p>This parameter is not sent to the motor controller until {@code setSetpoint()} is called.
     *
     * @param maxVolts The maximum (absolute) voltage value the remote PID will use.
     */
    public abstract void setMaxAbsControlEffortVolts(double maxVolts);

    /**
     * Returns the maximum control effort being currently used on the motor controller.
     *
     * @return The maximum (absolute) voltage value the remote PID can use.
     */
    public abstract double getMaxAbsControlEffortVolts();

    /**
     * Sets the maximum control effort of the PID's integral term in either direction, in volts.
     *
     * <p>This parameter is not sent to the motor controller until {@code setSetpoint()} is called.
     *
     * @param maxAccumulation The maximum (absolute) voltage value the remote PID's I will reach.
     */
    public abstract void setMaxIntegralAccumulationVolts(double maxAccumulation);

    /**
     * Returns the maximum control effort of the remote PID's I term.
     *
     * @return The maximum (absolute) voltage value the remote PID's I can use.
     */
    public abstract double getMaxIntegralAccumulationVolts();

    /**
     * Sets the setpoint of the remote PID controller.
     *
     * <p>Also flushes all locally configured parameters to the motor controller.
     *
     * @param setpoint The setpoint, of whatever the {@code RemoteControlType} of this controller
     *     is. Units in radians, seconds, amps.
     */
    public abstract void setSetpoint(double setpoint);

    @Override
    public void autoGenerateLogs(
            DataLog dataLog,
            String name,
            String subdirName,
            boolean publishToNT,
            boolean recordInLog) {
        if (!logsConstructed) {
            controlTypeLogger =
                    new Logger<>(
                            dataLog,
                            name + "/remoteControlType",
                            subdirName,
                            publishToNT,
                            recordInLog);

            LoggerFactory<Double> doubleFactory =
                    new LoggerFactory<>(dataLog, subdirName, publishToNT, recordInLog);

            appliedFeedforwardLogger = doubleFactory.getLogger(name + "/appliedFeedforward_volts");

            kPLogger = doubleFactory.getLogger(name + "/kP");
            kILogger = doubleFactory.getLogger(name + "/kI");
            kDLogger = doubleFactory.getLogger(name + "/kD");

            maxULogger = doubleFactory.getLogger(name + "/maxAbsControlEffort_volts");
            maxIntegralULogger =
                    doubleFactory.getLogger(name + "/maxAbsIntegralControlEffort_volts");

            logsConstructed = true;

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            controlTypeLogger.update(getControlType());

            appliedFeedforwardLogger.update(getFeedforwardVolts());

            kPLogger.update(getkP());
            kILogger.update(getkI());
            kDLogger.update(getkD());

            maxULogger.update(getMaxAbsControlEffortVolts());
            maxIntegralULogger.update(getMaxIntegralAccumulationVolts());
        }
    }
}
