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
package org.chsrobotics.lib.input;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Class to mock a joystick button from a hardware joystick axis. */
public class VirtualJoystickButton extends JoystickButton {
    /**
     * Constructs a VirtualJoystickButton.
     *
     * @param pollingLoop An EventLoop to poll for the resulting button's {@code Trigger}
     *     functionality.
     * @param axis The JoystickAxis to use as an input.
     * @param minimumValue The minimum value in [-1,1] (inclusive) the JoystickAxis can have for
     *     this to be considered pressed.
     * @param maximumValue The maximum value in [-1,1] (inclusive) the JoystickAxis can have for
     *     this to be considered pressed.
     * @param inverse If true, the button will be pressed *except* if it is between the minimum and
     *     maximum values.
     */
    public VirtualJoystickButton(
            EventLoop pollingLoop,
            JoystickAxis axis,
            double minimumValue,
            double maximumValue,
            boolean inverse) {
        super(
                pollingLoop,
                () -> {
                    if (!inverse) {
                        return (axis.getAsDouble() >= minimumValue
                                && axis.getAsDouble() <= maximumValue);
                    } else {
                        return !(axis.getAsDouble() >= minimumValue
                                && axis.getAsDouble() <= maximumValue);
                    }
                },
                ("virtual: " + axis.toString()),
                false);
    }

    /**
     * Constructs a VirtualJoystickButton, using the default CommandScheduler polling.
     *
     * @param axis The JoystickAxis to use as an input.
     * @param minimumValue The minimum value in [-1,1] (inclusive) the JoystickAxis can have for
     *     this to be considered pressed.
     * @param maximumValue The maximum value in [-1,1] (inclusive) the JoystickAxis can have for
     *     this to be considered pressed.
     * @param inverse If true, the button will be pressed *except* if it is between the minimum and
     *     maximum values.
     */
    public VirtualJoystickButton(
            JoystickAxis axis, double minimumValue, double maximumValue, boolean inverse) {
        this(
                CommandScheduler.getInstance().getDefaultButtonLoop(),
                axis,
                minimumValue,
                maximumValue,
                inverse);
    }
}
