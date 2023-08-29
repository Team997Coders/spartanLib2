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

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A group of ProfilePhases that represents an arbitrary trajectory of compound accelerations,
 * coasts, and decelerations.
 *
 * <p>A motion profile provides a trajectory, most often used as the setpoint for a PID (or similar)
 * controller.
 *
 * <p>This can be useful to avoid control effort saturation, or to decrease the initial large input
 * from a PID's P term. Can also be useful to remain within physical constraints of a mechanism.
 *
 * <p>This class only contains constructors out of user-defined and user-calculated phases, but
 * {@link AsymmetricTrapezoidProfile} and {@link TrapezoidProfile} extend this to create profiles
 * out of constraints and desired states.
 */
public class MotionProfile {
    protected List<ProfilePhase> phases = new ArrayList<>();
    protected State initialState;

    /** Holds the velocity and position states of a MotionProfile at a time in its span. */
    public static class State {
        /** Position of the profile at this state. */
        public final double position;

        /** Velocity of the profile at this state. */
        public final double velocity;

        /**
         * Constructs a State out of a given position and velocity.
         *
         * @param position Position of the profile.
         * @param velocity Velocity of the profile.
         */
        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean equals(Object other) {
            double epsilon = 0.0001;
            if (other instanceof State) {
                State rhs = (State) other;
                return Math.abs(this.position - rhs.position) < epsilon
                        && Math.abs(this.velocity - rhs.velocity) < epsilon;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(position, velocity);
        }

        @Override
        public String toString() {
            return "State[position: " + position + ", velocity: " + velocity + ", time: ";
        }
    }

    /**
     * Constructs a MotionProfile with arbitrary phases.
     *
     * @param initialState The initial state of the motion profile.
     * @param phases The ProfilePhases of the motion profile.
     */
    public MotionProfile(State initialState, ProfilePhase... phases) {
        this.initialState = initialState;
        for (ProfilePhase phase : phases) {
            this.phases.add(phase);
        }
    }

    /**
     * Constructs a MotionProfile with no phases and a defined initial state.
     *
     * <p>This constructor is protected because it encourages users to create empty profiles and
     * then add phases to the (also protected) {@code phases}, when this can cause undefined
     * behavior. This needs to exist, however, for certain subclasses of this to work properly.
     *
     * @param initialState The initial state of the motion profile.
     */
    protected MotionProfile(State initialState) {
        this.initialState = initialState;
    }

    /**
     * Constructs a MotionProfile of arbitrary phases with an initial state of zero displacement and
     * zero velocity.
     *
     * @param phases The ProfilePhases of the profile.
     */
    public MotionProfile(ProfilePhase... phases) {
        this(new State(0, 0), phases);
    }

    /**
     * Gets all the phases in the profile.
     *
     * @return An ordered list of ProfilePhases.
     */
    public List<ProfilePhase> getPhases() {
        return List.copyOf(phases);
    }

    /**
     * Calculates the current State of the profile at a given time.
     *
     * <p>If the time sampled is less than 0, returns the initial State. If it is greater than the
     * timespan of the profile, returns a State of the aggregated position and zero velocity.
     *
     * @param time The time since the beginning of the profile.
     * @return The position and velocity of the profile at that time.
     */
    public State sample(double time) {
        if (time <= 0) {
            return initialState;
        }
        double position = initialState.position;
        for (ProfilePhase phase : phases) {
            if (time - phase.time < 0) {
                return new State(
                        position
                                + 0.5 * phase.acceleration * Math.pow(time, 2)
                                + phase.initialVelocity * time,
                        time * phase.acceleration + phase.initialVelocity);
            } else {
                time -= phase.time;
                position += phase.position;
            }
        }
        // case where there are no phases, or the time is greater than the length of the profile
        return new State(position, 0);
    }

    /**
     * Returns the total time needed for the trajectory to finish.
     *
     * @return Total trajectory time.
     */
    public double totalTime() {
        double time = 0;

        for (ProfilePhase phase : phases) time += phase.time;

        return time;
    }
}
