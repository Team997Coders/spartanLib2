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
package org.chsrobotics.lib.trajectory.motionProfile;

import static org.junit.Assert.assertEquals;

import org.chsrobotics.lib.trajectory.motionProfile.MotionProfile.State;
import org.junit.Test;

/** Tests for the MotionProfile. */
public class MotionProfileTests {
    private static final double epsilon = 0.0001;

    @Test
    public void motionProfileStateEqualityWorks() {
        State state1 = new State(5, -2);
        State state2 = new State(5.0001, -1.9999);
        assertEquals(true, (state1.equals(state2)));
    }

    @Test
    public void motionProfileHandlesProfileWithNoPhases() {
        MotionProfile profile = new MotionProfile(new State(0, 0));

        assertEquals(new State(0, 0), profile.sample(5));
    }

    @Test
    public void motionProfileHandlesSampleTimeLessThanZero() {
        MotionProfile profile = new MotionProfile(new ProfilePhase(1, 1, 1));

        assertEquals(new State(0, 0), profile.sample(-10));
    }

    @Test
    public void motionProfileHandlesSampleTimeGreaterThanRuntime() {
        MotionProfile profile = new MotionProfile(new ProfilePhase(10, 10, 2));

        assertEquals(new State(40, 0), profile.sample(5));
    }

    @Test
    public void motionProfileProperlyAggregatesPhases() {
        ProfilePhase phase1 = new ProfilePhase(0, 5, 2);
        ProfilePhase phase2 = new ProfilePhase(-1, 5, 2);
        ProfilePhase phase3 = new ProfilePhase(0, 0, 3);

        MotionProfile profile = new MotionProfile(new State(0, 0), phase1, phase2, phase3);

        assertEquals(18, profile.sample(5).position, epsilon);
    }

    @Test
    public void motionProfileHandlesSamplingInMidpointOfPhase() {
        MotionProfile profile = new MotionProfile(new ProfilePhase(1, 0, 5));

        assertEquals(new State(8, 4), profile.sample(4));
    }
}
