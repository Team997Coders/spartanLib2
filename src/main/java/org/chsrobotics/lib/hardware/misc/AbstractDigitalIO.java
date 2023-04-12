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
package org.chsrobotics.lib.hardware.misc;

import edu.wpi.first.util.datalog.DataLog;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.telemetry.Logger.LoggerFactory;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public abstract class AbstractDigitalIO implements IntrinsicLoggable {
    public static enum IOMode {
        INPUT,
        OUTPUT
    }

    public static enum IOState {
        HIGH,
        LOW,
        NONE;

        public Boolean asBool() {
            if (this == HIGH) return true;
            else if (this == LOW) return false;
            else return null;
        }

        public static IOState fromBool(Boolean bool) {
            if (bool == null) return NONE;
            else if (bool) return HIGH;
            else return LOW;
        }
    }

    private IOState prevState = IOState.NONE;

    private boolean fallingEdge = false;
    private boolean risingEdge = false;

    private boolean logsConstructed = false;

    private Logger<IOMode> modeLogger;

    private Logger<IOState> outputLogger;
    private Logger<Boolean> outputAsBoolLogger;

    private Logger<IOState> inputLogger;
    private Logger<Boolean> inputAsBoolLogger;

    private Logger<Boolean> risingEdgeLogger;
    private Logger<Boolean> fallingEdgeLogger;

    public AbstractDigitalIO() {
        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public abstract IOState getInput();

    public abstract void setMode(IOMode mode);

    public abstract IOMode getCurrentMode();

    public abstract void setOutput(IOState state);

    public abstract IOState getCurrentOutput();

    public boolean getInputRisingEdge() {
        return risingEdge;
    }

    public boolean getInputFallingEdge() {
        return fallingEdge;
    }

    private void periodic() {
        if ((prevState == getInput())
                || (prevState == IOState.NONE)
                || (getInput() == IOState.NONE)) {
            fallingEdge = false;
            risingEdge = false;
        } else if (!prevState.asBool() && getInput().asBool()) {
            fallingEdge = false;
            risingEdge = true;
        } else if (prevState.asBool() && !getInput().asBool()) {
            fallingEdge = true;
            risingEdge = false;
        } else if (!prevState.asBool() && !getInput().asBool()) {
            fallingEdge = false;
            risingEdge = false;
        }
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {

            modeLogger = new Logger<>(log, name + "/ioMode", subdirName, publishToNT, recordInLog);

            LoggerFactory<IOState> ioStateFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            inputLogger = ioStateFactory.getLogger(name + "/input");

            outputLogger = ioStateFactory.getLogger(name + "/output");

            LoggerFactory<Boolean> boolFactory =
                    new LoggerFactory<>(log, subdirName, publishToNT, recordInLog);

            inputAsBoolLogger = boolFactory.getLogger(name + "/inputBool");
            outputAsBoolLogger = boolFactory.getLogger(name = "/outputBool");

            fallingEdgeLogger = boolFactory.getLogger(name + "/inputFallingEdge");
            risingEdgeLogger = boolFactory.getLogger(name + "/inputRisingEdge");

            PeriodicCallbackHandler.registerCallback(this::updateLogs);

            logsConstructed = true;
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            modeLogger.update(getCurrentMode());

            inputLogger.update(getInput());
            outputLogger.update(getCurrentOutput());

            inputAsBoolLogger.update(getInput().asBool());
            outputAsBoolLogger.update(getCurrentOutput().asBool());

            fallingEdgeLogger.update(getInputFallingEdge());
            risingEdgeLogger.update(getInputRisingEdge());
        }
    }
}
