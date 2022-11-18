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
package org.chsrobotics.lib.controllers;

import static org.junit.Assert.assertEquals;

import org.chsrobotics.lib.controllers.PID.PIDConstants;
import org.chsrobotics.lib.trajectory.AsymmetricTrapezoidProfile;
import org.chsrobotics.lib.trajectory.AsymmetricTrapezoidProfile.Constraints;
import org.chsrobotics.lib.trajectory.MotionProfile;
import org.chsrobotics.lib.trajectory.MotionProfile.State;
import org.chsrobotics.lib.trajectory.ProfilePhase;
import org.junit.Test;

/** Tests for the MotionProfilePID. */
public class MotionProfilePIDTests {
    private final double epsilon = 0.0001;

    @Test
    public void MotionProfilePIDPControllerWorks() {
        AsymmetricTrapezoidProfile profile =
                new AsymmetricTrapezoidProfile(new Constraints(1, 1, 1), new State(4, 0));
        MotionProfilePID controller = new MotionProfilePID(new PIDConstants(1, 0, 0), profile);

        assertEquals(0, controller.calculate(0, 0), epsilon);
        assertEquals(0.025, controller.calculate(0.1, 0.5), epsilon);
        assertEquals(-0.1, controller.calculate(0.6, 1), epsilon);
    }

    @Test
    public void MotionProfilePIDIntegralAccumulationWorks() {
        AsymmetricTrapezoidProfile profile =
                new AsymmetricTrapezoidProfile(new Constraints(5, 1, 1), new State(10, 0));
        MotionProfilePID controller = new MotionProfilePID(0, 0.5, 0, profile);

        controller.calculate(0, 0);
        controller.calculate(0.6, 1);
        controller.calculate(1.5, 2);
        controller.calculate(5.2, 3);

        assertEquals(-0.15, controller.calculate(0, 3), epsilon);
    }

    @Test
    public void MotionProfilePIDDerivativeWorks() {
        MotionProfile profile = new MotionProfile(new ProfilePhase(1, 0, 10));
        MotionProfilePID controller = new MotionProfilePID(0, 0, 2, profile);

        assertEquals(0, controller.calculate(0, 0), epsilon);
        assertEquals(0.5, controller.calculate(0, 0.5), epsilon);
        assertEquals(1.5, controller.calculate(0, 1), epsilon);
        // TODO: more D tests
    }
}
