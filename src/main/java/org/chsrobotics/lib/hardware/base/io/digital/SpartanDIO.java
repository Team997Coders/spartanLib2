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
package org.chsrobotics.lib.hardware.base.io.digital;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

// TODO docs
public class SpartanDIO implements AbstractDigitalInput, AbstractDigitalOutput {
    public static enum IOMode {
        INPUT,
        OUTPUT
    }

    public static record DigitalIOConfig(int channel, boolean inputInverted, IOMode initialMode) {
        public DigitalIOConfig setChannel(int channel) {
            return new DigitalIOConfig(channel, inputInverted, initialMode);
        }

        public DigitalIOConfig setInputInverted(boolean inverted) {
            return new DigitalIOConfig(channel, inverted, initialMode);
        }

        public DigitalIOConfig setInitialMode(IOMode initialMode) {
            return new DigitalIOConfig(channel, inputInverted, initialMode);
        }
    }

    private final DigitalIOConfig config;

    private DigitalInput input;

    private DigitalOutput output;

    private IOMode ioMode;

    private DigitalOutputState currentOutput = DigitalOutputState.NONE;

    private Logger<DigitalInputState> inputLogger;

    private Logger<DigitalOutputState> outputLogger;

    private Logger<IOMode> ioModeLogger;

    private boolean logsConstructed = false;

    public SpartanDIO(DigitalIOConfig config) {
        this.config = config;

        ioMode = config.initialMode;

        if (config.initialMode == IOMode.INPUT) input = new DigitalInput(config.channel);
        else output = new DigitalOutput(config.channel);
    }

    public DigitalIOConfig getConfig() {
        return config;
    }

    @Override
    public DigitalInputState getInput() {
        if (ioMode == IOMode.INPUT) {
            if (config.inputInverted) return DigitalInputState.fromBool(!input.get());
            else return DigitalInputState.fromBool(input.get());
        } else return DigitalInputState.NONE;
    }

    public void setMode(IOMode mode) {
        if (mode == IOMode.INPUT && ioMode != IOMode.INPUT) {
            output.close();

            input = new DigitalInput(config.channel);

            currentOutput = DigitalOutputState.NONE;

            ioMode = mode;
        } else if (mode == IOMode.OUTPUT && ioMode != IOMode.OUTPUT) {
            input.close();

            output = new DigitalOutput(config.channel);

            ioMode = mode;
        }
    }

    public IOMode getCurrentMode() {
        return ioMode;
    }

    public boolean setOutput(DigitalOutputState state) {
        if (ioMode == IOMode.OUTPUT && state.asBool() != null) {
            output.set(state.asBool());

            currentOutput = state;

            return true;
        } else return false;
    }

    @Override
    public DigitalOutputState getCurrentOutput() {
        return currentOutput;
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            inputLogger =
                    new Logger<>(log, name + "/inputState", subdirName, publishToNT, recordInLog);

            outputLogger =
                    new Logger<>(log, name + "/outputState", subdirName, publishToNT, recordInLog);

            ioModeLogger = new Logger<>(log, name + "ioMode", subdirName, publishToNT, recordInLog);

            logsConstructed = true;

            PeriodicCallbackHandler.registerCallback(this::updateLogs);
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            inputLogger.update(getInput());

            outputLogger.update(getCurrentOutput());

            ioModeLogger.update(getCurrentMode());
        }
    }
}
