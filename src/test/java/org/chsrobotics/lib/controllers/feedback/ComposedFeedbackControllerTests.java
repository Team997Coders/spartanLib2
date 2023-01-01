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
package org.chsrobotics.lib.controllers.feedback;

import static org.junit.Assert.assertEquals;

import java.util.Map;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.math.filters.NullFilter;
import org.junit.Test;

public class ComposedFeedbackControllerTests {
    private final double epsilon = 0.0001;

    @Test
    public void PControllerWorks() {
        ComposedFeedbackController controller =
                new ComposedFeedbackController(Map.of(new NullFilter(), 1.0));

        controller.setSetpoint(1);

        assertEquals(1, controller.calculate(0), epsilon);
        assertEquals(0.5, controller.calculate(0.5), epsilon);
        assertEquals(-0.1, controller.calculate(1.1), epsilon);
    }

    @Test
    public void MultipleFilterControllerWorks() {
        ComposedFeedbackController controller =
                new ComposedFeedbackController(
                        Map.of(new NullFilter(), 1.0, new DifferentiatingFilter(), 1.0));

        controller.setSetpoint(1);

        assertEquals(2, controller.calculate(0, 1), epsilon);
    }
}
