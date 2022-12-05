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
package org.chsrobotics.lib.drive.differential;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DifferentialDriveInputTests {
    @Test
    public void differentialMoveAdd() {
        DifferentialDriveInput actual =
                new DifferentialDriveInput(0.5, 0.5).add(new DifferentialDriveInput(0.5, -0.2));
        DifferentialDriveInput expected = new DifferentialDriveInput(1.0, 0.3);
        assertEquals(actual, expected);
    }

    @Test
    public void differentialMoveMultiply() {
        DifferentialDriveInput actual = new DifferentialDriveInput(1.0, 0.0).multiply(-0.5);
        DifferentialDriveInput expected = new DifferentialDriveInput(-0.5, 0.0);
        assertEquals(actual, expected);
    }
}
