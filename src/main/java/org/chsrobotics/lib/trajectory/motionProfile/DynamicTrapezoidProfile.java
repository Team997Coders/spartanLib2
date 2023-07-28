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

/**
 * TODO
 *
 * <p>A MotionProfile that allows for smooth motion between two states, generated with acceleration,
 * velocity, and deceleration constraints.
 *
 * <p>The dx/dt of this motion (velocity) forms the trapezoid shape on a graph. The particular
 * elements of motion are velocity and acceleration limited, with acceleration limiting having
 * different constraints for the initial phase of motion ("maxAcceleration"), and the final phase of
 * motion ("maxDeceleration").
 *
 * <p>The most useful practical application of this is use defining the setpoint for a PID (or
 * similar) controller, to avoid control effort saturation or the initial spike in input from the P
 * term.
 *
 * <p>Also could be used to meet physical constraints from a mechanism (can't go too fast or the
 * wires might tangle, has to decelerate at a different rate depending on the location in its track,
 * etc...).
 */
public class DynamicTrapezoidProfile {

    /**
     * TODO docs A data class to hold the maximum allowable rates for the output of a
     * TrapezoidProfile.
     *
     * <p>Because of the number of cases this profile has to handle, acceleration and deceleration
     * don't coorespond to acceleration in a specific direction, or towards higher/lower absolute
     * values. It's probably best to think of them as the acceleration constraint for the first
     * phase of motion, and deceleration constraint for the final phase of motion.
     *
     * @param maxVelocity TODO
     * @param maxAcceleration
     */
    public static record DynamicTrapezoidProfileConstraints(
            double maxVelocity, double maxAcceleration) {}
    ;

    private final DynamicTrapezoidProfileConstraints constraints;

    /**
     * Constructs a DynamicTrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     */
    public DynamicTrapezoidProfile(DynamicTrapezoidProfileConstraints constraints) {
        this.constraints = constraints;
    }

    public MotionProfileState getNextSetpoint(
            double targetPosition, MotionProfileState current, double controlPeriod) {
        // check if the current position is already at the target position
        if (current.position() == targetPosition) {
            return new MotionProfileState(current.position(), 0);
        }

        // calculate the distance to the target and the direction to it
        double distanceToTarget = targetPosition - current.position();
        int directionToTarget = (distanceToTarget >= 0) ? 1 : -1;

        // time to stop from the current velocity
        double stopTime = Math.abs(current.velocity()) / constraints.maxAcceleration;

        // distance covered between now and rest in a stop state
        double stopAccumulation =
                0.5 * directionToTarget * constraints.maxAcceleration * stopTime * stopTime;

        // speculative next state if acceleration is chosen
        double futureDistanceToTarget =
                targetPosition
                        - (current.velocity() * controlPeriod)
                        - (0.5
                                * constraints.maxAcceleration
                                * directionToTarget
                                * controlPeriod
                                * controlPeriod)
                        - current.position();

        double acceleration;

        // check if it is possible to stop before reaching the target with full slowing
        if (Math.abs(stopAccumulation) >= Math.abs(futureDistanceToTarget)) {
            acceleration = -directionToTarget * constraints.maxAcceleration;
        } else {
            acceleration = directionToTarget * constraints.maxAcceleration;

            // velocity constraint
            if (Math.abs(current.velocity() + (acceleration * controlPeriod))
                    > constraints.maxVelocity) {
                acceleration = 0;
            }
        }

        // integrate forwards for next state
        double nextPosition =
                (0.5 * acceleration * controlPeriod * controlPeriod)
                        + (controlPeriod * current.velocity())
                        + current.position();
        double nextVelocity = (controlPeriod * acceleration) + current.velocity();

        // stop overshoots
        if ((nextPosition - targetPosition) * directionToTarget > 0) {
            nextPosition = targetPosition;
            nextVelocity = 0;
        }

        return new MotionProfileState(nextPosition, nextVelocity);
    }
}
