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
package org.chsrobotics.lib.controllers.feedback;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Tests for the SpartanLib PID controller. */
public class PIDTests {
    private final double epsilon = 0.0001;

    @Test
    public void PIDProportionalControllerWorks() {
        PID controller = new PID(1, 0, 0, 0, 0);
        controller.setSetpoint(5);
        assertEquals(5, controller.calculate(0), epsilon);
        assertEquals(2.5, controller.calculate(2.5), epsilon);
        assertEquals(0.1, controller.calculate(4.9), epsilon);
        assertEquals(-0.5, controller.calculate(5.5), epsilon);
    }

    @Test
    public void PIDIntegralControllerWorks() {
        PID controller = new PID(0, 0.1, 0, 0, 10);
        assertEquals(1, controller.calculate(0, 1), epsilon);
        assertEquals(1.9, controller.calculate(1, 1), epsilon);
        assertEquals(2.5, controller.calculate(4, 1), epsilon);

        controller.resetIntegralAccumulation();
        assertEquals(0, controller.getIntegralAccumulation(), 0);

        controller.setkI(2.5);
        assertEquals(0.5, controller.calculate(0), epsilon);
        assertEquals(0.75, controller.calculate(5), epsilon);
        assertEquals(0.5, controller.calculate(15), epsilon);
    }

    @Test
    public void PIDDerivativeControllerWorks() {
        PID controller = new PID(0, 0, 0.1, 0, 10);
        assertEquals(0, controller.calculate(0, 1), epsilon);
        assertEquals(-0.5, controller.calculate(5, 1), epsilon);
        assertEquals(0.1, controller.calculate(4, 1), epsilon);
        assertEquals(-0.8, controller.calculate(12, 1), epsilon);

        controller.resetPreviousMeasurement();
        controller.setkD(2);
        assertEquals(-100, controller.calculate(1), epsilon);
        assertEquals(0, controller.calculate(1), epsilon);
    }

    @Test
    public void PIDAtSetpointWorks() {
        PID controller = new PID(0, 0, 0, 0, 100);

        controller.calculate(98);
        assertEquals(false, controller.atSetpoint());
        controller.calculate(99, 1);
        assertEquals(true, controller.atSetpoint());

        controller.setSetpointTolerance(0.01, 0.01);
        assertEquals(false, controller.atSetpoint());
        controller.calculate(99.5, 1);
        assertEquals(true, controller.atSetpoint());
    }
}
