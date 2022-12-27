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

import edu.wpi.first.wpilibj.event.EventLoop;
import org.junit.Test;

public class VirtualJoystickButtonTests {
    private double value = 0;

    @Test
    /** Tests for the VirtualJoystickButton. */
    public void VirtualJoystickButtonWorksAsExpected() {
        JoystickAxis axis = new JoystickAxis(() -> value, "test", false);

        VirtualJoystickButton buttonA =
                new VirtualJoystickButton(new EventLoop(), axis, -0.5, 0.5, false);
        VirtualJoystickButton buttonB =
                new VirtualJoystickButton(new EventLoop(), axis, -0.5, 0.5, true);

        value = 0.5;
        assertEquals(true, buttonA.getAsBoolean());
        assertEquals(false, buttonB.getAsBoolean());

        value = 0.5001;
        assertEquals(false, buttonA.getAsBoolean());
        assertEquals(true, buttonB.getAsBoolean());

        value = -0.5;
        assertEquals(true, buttonA.getAsBoolean());
        assertEquals(false, buttonB.getAsBoolean());

        value = -0.5001;
        assertEquals(false, buttonA.getAsBoolean());
        assertEquals(true, buttonB.getAsBoolean());

        value = -0.5;
        assertEquals(true, buttonA.getAsBoolean());
        assertEquals(false, buttonB.getAsBoolean());

        value = 0;
        assertEquals(true, buttonA.getAsBoolean());
        assertEquals(false, buttonB.getAsBoolean());

        value = 1;
        assertEquals(false, buttonA.getAsBoolean());
        assertEquals(true, buttonB.getAsBoolean());
    }
}
