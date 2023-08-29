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
package org.chsrobotics.lib.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/* Tests for the GearRatioHelper. **/
public class GearRatioHelperTests {
    private static final double epsilon = 0.0001;

    @Test
    public void gearRatioHelperInverseProperly() {
        GearRatioHelper helper = new GearRatioHelper(5, 1);
        assertEquals(87.3, helper.inputFromOutput(helper.outputFromInput(87.3)), epsilon);
    }

    @Test
    public void gearRatioHelperOutputToInput() {
        GearRatioHelper helper = new GearRatioHelper(3, 5);
        assertEquals(12, helper.outputFromInput(20), epsilon);
    }

    @Test
    public void gearRatioHelperInputToOutput() {
        GearRatioHelper helper = new GearRatioHelper(7, 1);
        assertEquals(5, helper.inputFromOutput(35), epsilon);
    }

    @Test
    public void GearRatioHelperAddReduction() {
        GearRatioHelper helper = new GearRatioHelper(1, 5);

        GearRatioHelper reductionStage = new GearRatioHelper(1, 2);

        assertEquals(12.5, helper.addStage(reductionStage).outputFromInput(125), epsilon);
    }

    @Test
    public void GearRatioHelperAddStepUp() {
        GearRatioHelper helper = new GearRatioHelper(2, 1);

        GearRatioHelper stepUpStage = new GearRatioHelper(3, 2);

        assertEquals(27, helper.addStage(stepUpStage).outputFromInput(9), epsilon);
    }
}
