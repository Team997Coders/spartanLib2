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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class SpartanDigitalIO extends AbstractDigitalIO {
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

    private IOState currentOutput = IOState.NONE;

    public SpartanDigitalIO(DigitalIOConfig config) {
        this.config = config;

        ioMode = config.initialMode;

        if (config.initialMode == IOMode.INPUT) input = new DigitalInput(config.channel);
        else output = new DigitalOutput(config.channel);
    }

    public DigitalIOConfig getConfig() {
        return config;
    }

    @Override
    public IOState getInput() {
        if (ioMode == IOMode.INPUT) {
            if (config.inputInverted) return IOState.fromBool(!input.get());
            else return IOState.fromBool(input.get());
        } else return IOState.NONE;
    }

    @Override
    public void setMode(IOMode mode) {
        if (mode == IOMode.INPUT && ioMode != IOMode.INPUT) {
            output.close();

            input = new DigitalInput(config.channel);

            currentOutput = IOState.NONE;

            ioMode = mode;
        } else if (mode == IOMode.OUTPUT && ioMode != IOMode.OUTPUT) {
            input.close();

            output = new DigitalOutput(config.channel);

            ioMode = mode;
        }
    }

    @Override
    public IOMode getCurrentMode() {
        return ioMode;
    }

    @Override
    public void setOutput(IOState state) {
        if (ioMode == IOMode.OUTPUT && state.asBool() != null) {
            output.set(state.asBool());

            currentOutput = state;
        }
    }

    @Override
    public IOState getCurrentOutput() {
        return currentOutput;
    }
}
