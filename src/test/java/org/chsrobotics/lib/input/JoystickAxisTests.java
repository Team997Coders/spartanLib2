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

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

/** Tests for the JoystickAxis. */
public class JoystickAxisTests {
    private double value;

    private final JoystickAxis axis = new JoystickAxis(() -> value, "test", false);

    @Before
    public void reset() {
        value = 0;
        axis.getAsDouble();
    }

    @Test
    public void JoystickAxisGetValueWorks() {
        assertEquals(0, axis.getAsDouble(), 0);

        value = 0.2;
        assertEquals(0.2, axis.getAsDouble(), 0);

        value = -0.8;
        assertEquals(-0.8, axis.getAsDouble(), 0);
    }

    @Test
    public void JoystickAxisDeadbandWorks() {
        axis.addDeadband(0.1);
        value = 0.05;
        assertEquals(0, axis.getAsDouble(), 0);
        value = -0.1;
        assertEquals(0, axis.getAsDouble(), 0);

        axis.addDeadband(-0.5);
        value = 0.45;
        assertEquals(0, axis.getAsDouble(), 0);
        value = -0.55;
        assertEquals(-0.55, axis.getAsDouble(), 0);
    }
}
