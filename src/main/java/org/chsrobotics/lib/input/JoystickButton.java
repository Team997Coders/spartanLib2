/**
Copyright 2022 FRC Team 997

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

import java.util.function.Supplier;

/** Represents a hardware button on an input device which has two states. */
public class JoystickButton {
    private final Supplier<Boolean> pressedLambda;
    private boolean lastState = false;
    private final String name;
    private final boolean isReal;

    /**
     * Constructs a JoystickButton.
     *
     * @param pressedLambda Supplier of the boolean state of the button.
     * @param name String identifier of the index of this button in the driver station view.
     * @param isReal If this JoystickButton is a representation of actual hardware.
     */
    protected JoystickButton(Supplier<Boolean> pressedLambda, String name, boolean isReal) {
        this.pressedLambda = pressedLambda;
        this.name = name;
        this.isReal = isReal;
    }

    /**
     * Returns a boolean of whether the button is pressed (in a {@code true} state).
     *
     * @return Whether the button is pressed.
     */
    public boolean isPressed() {
        lastState = pressedLambda.get();
        return pressedLambda.get();
    }

    /**
     * Returns whether the button has changed state since the last time {@code isPressed} was
     * called.
     *
     * @return Whether the state has changed from true to false or false to true.
     */
    public boolean hasChangedFromPreviousValue() {
        return (pressedLambda.get() != lastState);
    }

    /**
     * Whether this JoystickButton is a representation of physical hardware.
     *
     * @return If this exists on an actual controller.
     */
    public boolean isReal() {
        return isReal;
    }

    @Override
    public String toString() {
        return ("JoystickButton: " + name);
    }
}
