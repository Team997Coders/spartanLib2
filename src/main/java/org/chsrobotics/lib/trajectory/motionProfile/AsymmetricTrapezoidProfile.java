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

import java.util.Objects;

/**
 * A MotionProfile that allows for smooth motion between two states, generated with acceleration,
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
public class AsymmetricTrapezoidProfile extends MotionProfile {

    /**
     * A data class to hold the maximum allowable rates for the output of a
     * AsymmetricTrapezoidProfile.
     *
     * <p>Because of the number of cases this profile has to handle, acceleration and deceleration
     * don't coorespond to acceleration in a specific direction, or towards higher/lower absolute
     * values. It's probably best to think of them as the acceleration constraint for the first
     * phase of motion, and deceleration constraint for the final phase of motion.
     */
    public static class Constraints {
        public final double maxVelocity;
        public final double maxAcceleration;
        public final double maxDeceleration;

        /**
         * Constructs Constraints for a AsymmetricTrapezoidProfile.
         *
         * @param maxVelocity maximum velocity.
         * @param maxAcceleration maximum acceleration.
         * @param maxDeceleration maximum deceleration.
         */
        public Constraints(double maxVelocity, double maxAcceleration, double maxDeceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;
        }

        @Override
        public boolean equals(Object other) {
            double epsilon = 0.0001;
            if (other instanceof Constraints) {
                Constraints rhs = (Constraints) other;
                return Math.abs(this.maxVelocity - rhs.maxVelocity) < epsilon
                        && Math.abs(this.maxAcceleration - rhs.maxAcceleration) < epsilon
                        && Math.abs(this.maxDeceleration - rhs.maxDeceleration) < epsilon;
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(maxVelocity, maxAcceleration, maxDeceleration);
        }

        @Override
        public String toString() {
            return "AsymmetricTrapezoidProfileConstraints[maxVelocity: "
                    + maxVelocity
                    + ", maxAcceleration: "
                    + maxAcceleration
                    + ", maxDeceleration"
                    + maxDeceleration
                    + "]";
        }
    }

    /**
     * Constructs an AsymmetricTrapezoidProfile with an initial position and velocity of 0,0.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param target The desired state when the profile is complete.
     */
    public AsymmetricTrapezoidProfile(Constraints constraints, State target) {
        this(constraints, target, new State(0, 0));
    }

    /**
     * Constructs an AsymmetricTrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param target The desired state when the profile is complete.
     * @param initial The initial state (usually the current state).
     */
    public AsymmetricTrapezoidProfile(Constraints constraints, State target, State initial) {
        super(initial);

        // in the case the target position is before the initial, we should calculate it all
        // positive,
        // and flip the sign of the resulting acceleration and position
        double targetPosition = target.position - initial.position;
        int direction = targetPosition < 0 ? -1 : 1;

        double maxVelocity = constraints.maxVelocity * direction;
        double maxAccel = Math.abs(constraints.maxAcceleration) * direction;
        double maxDecel = Math.abs(constraints.maxDeceleration) * direction * -1;

        // constrain the initial and target velocities to bellow maxVelocity (this is how wpilib
        // does it)
        // initial velocity can be in the wrong direction when starting
        double initialVelocity =
                direction == 1
                        ? Math.min(initial.velocity, maxVelocity)
                        : Math.max(initial.velocity, maxVelocity);
        // target velocity will always be in the correct direction
        double targetVelocity =
                direction == 1
                        ? Math.min(target.velocity, maxVelocity)
                        : Math.max(target.velocity, maxVelocity);

        initialState = new State(initial.position, initialVelocity);

        // calculate the time and position to reach max velocity (don't worry if we go over max
        // position)
        double accelTime = (maxVelocity - initialVelocity) / maxAccel;
        double accelPos = 0.5 * maxAccel * Math.pow(accelTime, 2) + accelTime * initialVelocity;

        // calculate the time and position to decelerate to target velocity
        double decelTime = (targetVelocity - maxVelocity) / maxDecel;
        double decelPos = -0.5 * maxDecel * Math.pow(decelTime, 2) + decelTime * initialVelocity;

        // the remaining position is at max velocity (coasting)
        double coastPos = targetPosition - (accelPos + decelPos);
        double coastTime = coastPos / maxVelocity;

        // if the coast position is a different sign then the rest of the phases, then it's not
        // possible to speed up to
        // the max velocity without overshooting, so we need to find the intersection of the
        // acceleration and deceleration.
        if (coastPos * direction < 0) {
            /*
             * We calculate the time of intersection by splitting up the profile into 3 integrals: the acceleration to
             * the intersection point, the deceleration back to the initial velocity, then the deceleration to the
             * target velocity (if the target velocity is greater than the initial velocity this will still work as the
             * integral of this will be negative). The integral of the profile (the sum of these 3 integrals) is equal
             * to the total position. If we evaluate each of these integrals from 0 to the time for each integral
             * (accelTime, decelTime, endTime) we can then use the fact that accel*accelTime = -d*decelTime and
             * endTime = -(initialVelocity - targetVelocity)/decel to put everything in terms of either accelTime or
             * constants, then we can use the quadratic formula to solve for accelTime
             */
            double vDiff = initialVelocity - targetVelocity;
            double a = (0.5 * maxAccel) - (Math.pow(maxAccel, 2) / (2 * maxDecel));
            double b = initialVelocity - ((initialVelocity * maxAccel) / maxDecel);
            double c =
                    -((Math.pow(vDiff, 2) / (2 * maxDecel))
                            + ((targetVelocity * vDiff) / maxDecel)
                            + targetPosition);
            // subtracting the square root will always produce a negative result with positive
            // target positions,
            // and vice versa with negatives, so we can always add direction * sqrt.
            accelTime = (-b + direction * Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
            decelTime = -((maxAccel / maxDecel) * accelTime + vDiff / maxDecel);

            // recalculate accel and decel position based off new times
            accelPos = 0.5 * maxAccel * Math.pow(accelTime, 2) + accelTime * initialVelocity;
            decelPos = -0.5 * maxDecel * Math.pow(decelTime, 2) + decelTime * targetVelocity;

            // since we've adjusted the accel and decel we need to set the coast to 0
            coastTime = 0;
            coastPos = 0;

            // if the deceleration time is negative, then our target velocity is higher than we can
            // possibly get to
            // it's probably best to just go as fast as we can and end in the right position
            if (decelTime < 0) {
                accelPos = targetPosition;
                accelTime =
                        (-initialVelocity
                                        + Math.sqrt(
                                                Math.pow(initialVelocity, 2)
                                                        + 2 * maxAccel * accelPos))
                                / maxAccel;
            }
            // if the acceleration time is negative, then our target velocity is lower than we can
            // possibly get to,
            // and we should decelerate faster than our maximum deceleration so that we get to our
            // position
            // at the target velocity
            if (accelTime < 0) {
                decelPos = targetPosition;
                decelTime = (2 * decelPos) / (initialVelocity + targetVelocity);
                maxDecel = (targetVelocity - initialVelocity) / decelTime;
                // we set these so that our initial velocity in the ProfilePhase is correct
                accelTime = -1;
                maxAccel = 0;
            }
        }

        ProfilePhase accelPhase = new ProfilePhase(accelTime, accelPos, maxAccel, initialVelocity);
        ProfilePhase coastPhase = new ProfilePhase(coastTime, coastPos, 0, maxVelocity);
        ProfilePhase decelPhase =
                new ProfilePhase(
                        decelTime, decelPos, maxDecel, accelTime * maxAccel + initialVelocity);

        if (accelPhase.time > 0) super.phases.add(accelPhase);
        if (coastPhase.time > 0) super.phases.add(coastPhase);
        if (decelPhase.time > 0) super.phases.add(decelPhase);
    }
}
