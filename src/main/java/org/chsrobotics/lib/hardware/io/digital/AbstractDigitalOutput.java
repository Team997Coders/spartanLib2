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
package org.chsrobotics.lib.hardware.io.digital;

import org.chsrobotics.lib.telemetry.IntrinsicLoggable;

public interface AbstractDigitalOutput extends IntrinsicLoggable {
    public static enum DigitalOutputState {
        HIGH,
        LOW,
        NONE;

        public Boolean asBool() {
            if (this == DigitalOutputState.NONE) return null;
            else if (this == DigitalOutputState.HIGH) return true;
            else return false;
        }

        public static DigitalOutputState fromBoolean(Boolean bool) {
            if (bool == null) return DigitalOutputState.NONE;
            if (bool) return DigitalOutputState.HIGH;
            else return DigitalOutputState.LOW;
        }
    }

    public boolean setOutput(DigitalOutputState state);

    public DigitalOutputState getCurrentOutput();
}
