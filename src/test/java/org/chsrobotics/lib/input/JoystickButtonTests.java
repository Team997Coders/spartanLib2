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

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

/** Tests for the JoystickButton. */
public class JoystickButtonTests {
    private boolean value;
    private final JoystickButton button = new JoystickButton(() -> value, "test", false);

    @Before
    public void reset() {
        value = false;
        button.getAsBoolean();
    }

    @Test
    public void JoystickButtonIsPressedWorks() {
        assertEquals(false, button.getAsBoolean());

        value = true;
        assertEquals(true, button.getAsBoolean());
    }
}
