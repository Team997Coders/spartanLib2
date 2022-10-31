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
package frc.team997.lib.trajectory;

import static org.junit.Assert.assertEquals;

import frc.team997.lib.trajectory.MotionProfile.State;
import org.junit.Test;

public class MotionProfileTests {
    private final double epsilon = 0.0001;

    @Test
    public void MotionProfileStateEqualityWorks() {
        State state1 = new State(5, -2);
        State state2 = new State(5.0001, -1.9999);
        assertEquals(true, (state1.equals(state2)));
    }

    @Test
    public void MotionProfileHandlesProfileWithNoPhases() {
        MotionProfile profile = new MotionProfile(new State(0, 0));

        assertEquals(new State(0, 0), profile.calculate(5));
    }

    @Test
    public void MotionProfileHandlesSampleTimeLessThanZero() {
        MotionProfile profile = new MotionProfile(ProfilePhase.fromRatesAndTime(1, 1, 1));

        assertEquals(new State(0, 0), profile.calculate(-10));
    }

    @Test
    public void MotionProfileHandlesSampleTimeGreaterThanRuntime() {
        MotionProfile profile = new MotionProfile(ProfilePhase.fromRatesAndTime(10, 10, 2));

        assertEquals(new State(40, 0), profile.calculate(5));
    }

    @Test
    public void MotionProfileProperlyAggregatesPhases() {
        ProfilePhase phase1 = ProfilePhase.fromRatesAndTime(0, 5, 2);
        ProfilePhase phase2 = ProfilePhase.fromRatesAndTime(-1, 5, 2);
        ProfilePhase phase3 = ProfilePhase.fromRatesAndTime(0, 0, 3);

        MotionProfile profile = new MotionProfile(new State(0, 0), phase1, phase2, phase3);

        assertEquals(18, profile.calculate(5).position, epsilon);
    }

    @Test
    public void MotionProfileHandlesSamplingInMidpointOfPhase() {
        MotionProfile profile = new MotionProfile(ProfilePhase.fromRatesAndTime(1, 0, 5));

        assertEquals(new State(8, 4), profile.calculate(4));
    }
}
