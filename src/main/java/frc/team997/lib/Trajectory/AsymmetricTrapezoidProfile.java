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
package frc.team997.lib.Trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * An asymmetric trapezoid-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on trapezoidal velocity constraints. To compute the
 * reference obeying this constraint, do the following.
 *
 * <p>Initialization:
 *
 * <pre><code>
 * AsymmetricTrapezoidProfile.Constraints constraints =
 *   new AsymmetricTrapezoidProfile.Constraints(kMaxV, kMaxA, kMaxD);
 * AsymmetricTrapezoidProfile.State previousProfiledReference =
 *   new AsymmetricTrapezoidProfile.State(initialReference, 0.0);
 * </code></pre>
 *
 * <p>Run on update:
 *
 * <pre><code>
 * AsymmetricTrapezoidProfile profile =
 *   new AsymmetricTrapezoidProfile(constraints, unprofiledReference, previousProfiledReference);
 * previousProfiledReference = profile.calculate(timeSincePreviousUpdate);
 * </code></pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when the unprofiled
 * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
 * determine when the profile has completed via `isFinished()`.
 */
public class AsymmetricTrapezoidProfile {
    // holds all the phases in order
    public final List<Phase> phases = new ArrayList<>(3);
    public final Constraints constraints;
    public final State initial;
    public final State target;
    public final double direction;

    public static class Constraints {
        final double maxVelocity;
        final double maxAcceleration;
        final double maxDeceleration;
        /**
         * Construct constraints for a AsymmetricTrapezoidProfile.
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
            return "Constraints[maxVelocity: "+maxVelocity+", maxAcceleration: "+maxAcceleration+", maxDeceleration"+maxDeceleration+"]";
        }
    }
    public static class State {
        final double position;
        final double velocity;
        /**
         * Construct a state for a AsymmetricTrapezoidProfile.
         *
         * @param position state position.
         * @param velocity state velocity.
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
            return "State[position: "+position+", velocity: "+velocity+"]";
        }
    }

    /**
     * A data class for profile phase data.
     */
    public static class Phase {
        final double time;
        final double position;
        final double acceleration;
        final double initialVelocity;

        /**
         * Construct a Phase.
         *
         * @param time The duration of the phase.
         * @param position The displacement of the phase.
         * @param acceleration The acceleration of the phase (0 if coast phase).
         * @param initialVelocity The velocity at the beggining of a phase.
         */
        public Phase(double time, double position, double acceleration, double initialVelocity) {
            this.time = time;
            this.position = position;
            this.acceleration = acceleration;
            this.initialVelocity = initialVelocity;
        }
        @Override
        public boolean equals(Object other) {
            double epsilon = 0.0001;
            if (other instanceof Phase) {
                Phase rhs = (Phase) other;
                return this.time == rhs.time
                        && Math.abs(this.position - rhs.position) < epsilon
                        && Math.abs(this.acceleration - rhs.acceleration) < epsilon
                        && Math.abs(this.initialVelocity - rhs.initialVelocity) < epsilon;
            } else {
                return false;
            }
        }
        @Override
        public int hashCode() {
            return Objects.hash(time, position, acceleration, initialVelocity);
        }
        @Override
        public String toString() {
            return "Phase[time: "+time+", position: "+position+", acceleration: "+acceleration+", initialVelocity:"+initialVelocity+"]";
        }
    }

    /**
     * Construct an AsymmetricTrapezoidProfile with an initial state of 0,0.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param target The desired state when the profile is complete.
     */
    public AsymmetricTrapezoidProfile(Constraints constraints, State target) {
        this(constraints, target, new State(0, 0));
    }
    /**
     * Construct an AsymmetricTrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param target The desired state when the profile is complete.
     * @param initial The initial state (usually the current state).
     */
    public AsymmetricTrapezoidProfile(Constraints constraints, State target, State initial) {
        // in the case the target position is before the initial, we should calculate it all positive,
        // and flip the sign of the resulting acceleration and position
        double targetPosition = target.position - initial.position;
        direction = targetPosition < 0 ? -1 : 1;

        double maxVelocity = constraints.maxVelocity * direction;
        double maxAccel = Math.abs(constraints.maxAcceleration) * direction;
        double maxDecel = Math.abs(constraints.maxDeceleration) * direction * -1;

        this.constraints = new Constraints(maxVelocity, maxAccel, maxDecel);

        // constrain the initial and target velocities to bellow maxVelocity (this is how wpilib does it)
        // initial velocity can be in the wrong direction when starting
        double initialVelocity = direction == 1 ? Math.min(initial.velocity, maxVelocity) : Math.max(initial.velocity, maxVelocity);
        // target velocity will always be in the correct direction
        double targetVelocity = direction == 1 ? Math.min(target.velocity, maxVelocity) : Math.max(target.velocity, maxVelocity);

        this.initial = new State(initial.position, initialVelocity);
        this.target = new State(target.position, targetVelocity);

        // calculate the time and position to reach max velocity (don't worry if we go over max position)
        double accelTime = (maxVelocity-initialVelocity)/maxAccel;
        double accelPos = 0.5 * maxAccel * Math.pow(accelTime, 2) + accelTime*initialVelocity;

        // calculate the time and position to decelerate to target velocity
        double decelTime = (targetVelocity-maxVelocity)/maxDecel;
        double decelPos = -0.5 * maxDecel * Math.pow(decelTime, 2) + decelTime*initialVelocity;

        // the remaining position is at max velocity (coasting)
        double coastPos = targetPosition - (accelPos + decelPos);
        double coastTime = coastPos / maxVelocity;

        // if the coast position is a different sign then the rest of the phases, then it's not possible to speed up to
        // the max velocity without overshooting, so we need to find the intersection of the acceleration and deceleration.
        if (coastPos*direction < 0) {
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
            double vDiff = initialVelocity-targetVelocity;
            double a = (0.5*maxAccel) - (Math.pow(maxAccel, 2)/(2*maxDecel));
            double b = initialVelocity - ((initialVelocity*maxAccel)/maxDecel);
            double c = -((Math.pow(vDiff, 2)/(2*maxDecel)) + ((targetVelocity*vDiff)/maxDecel) + targetPosition);
            // subtracting the square root will always produce a negative result with positive target positions,
            // and vice versa with negatives, so we can always add direction * sqrt.
            accelTime = (-b + direction*Math.sqrt(Math.pow(b, 2)-(4*a*c)))/(2*a);
            decelTime = -((maxAccel/maxDecel)*accelTime + vDiff/maxDecel);

            // recalculate accel and decel position based off new times
            accelPos = 0.5 * maxAccel * Math.pow(accelTime, 2) + accelTime*initialVelocity;
            decelPos = -0.5 * maxDecel * Math.pow(decelTime, 2) + decelTime*targetVelocity;

            // since we've adjusted the accel and decel we need to set the coast to 0
            coastTime = 0;
            coastPos = 0;

            // if the deceleration time is negative, then our target velocity is higher than we can possibly get to
            // it's probably best to just go as fast as we can and end in the right position
            if (decelTime < 0) {
                accelPos = targetPosition;
                accelTime = (-initialVelocity + Math.sqrt(Math.pow(initialVelocity, 2) + 2*maxAccel*accelPos))/maxAccel;
            }
            // if the acceleration time is negative, then our target velocity is lower than we can possibly get to,
            // and we should decelerate faster than our maximum deceleration so that we get to our position
            // at the target velocity
            if (accelTime < 0) {
                decelPos = targetPosition;
                decelTime = (2*decelPos)/(initialVelocity+targetVelocity);
                maxDecel = (targetVelocity-initialVelocity)/decelTime;
            }
        }

        Phase accelPhase = new Phase(accelTime, accelPos, maxAccel, initialVelocity);
        Phase coastPhase = new Phase(coastTime, coastPos, 0, maxVelocity);
        Phase decelPhase = new Phase(decelTime, decelPos, maxDecel, accelTime*maxAccel+initialVelocity);

        if (accelPhase.time > 0)
            phases.add(accelPhase);
        if (coastPhase.time > 0)
            phases.add(coastPhase);
        if (decelPhase.time > 0)
            phases.add(decelPhase);
    }
    /**
     * Calculate the correct position and velocity for the profile at a given time
     *
     * @param time The time since the beginning of the profile.
     * @return The position and velocity of the profile at the time.
     */
    public State calculate(double time) {
        double position = initial.position;
        for (Phase phase : phases) {
            if (time-phase.time < 0) {
                return new State(
                        position + 0.5 * phase.acceleration * Math.pow(time, 2) + phase.initialVelocity*time,
                        time*phase.acceleration + phase.initialVelocity
                );
            } else {
                time-=phase.time;
                position+=phase.position;
            }
        }
        return new State(position, target.velocity);
    }
    /**
     * Returns the time left until a target distance in the profile is reached.
     *
     * @param targetPosition The target position.
     * @return The time left until a target position in the profile is reached.
     * Returns NaN if targetPosition < initial.position, and the total position if targetPosition > target.position
     */
    public double timeLeftUntil(double targetPosition) {
        targetPosition = targetPosition - initial.position;
        double time = 0;
        for (Phase phase : phases) {
            if ((targetPosition-phase.position)*direction < 0) {
                if (phase.acceleration==0) {
                    time += targetPosition/constraints.maxVelocity;
                } else {
                    double sqrt = Math.sqrt(Math.pow(phase.initialVelocity, 2)+2*phase.acceleration*targetPosition);
                    time += (-phase.initialVelocity + direction*sqrt)/phase.acceleration;
                }
                return time;
            } else {
                time += phase.time;
                targetPosition-=phase.position;
            }
        }
        return time;
    }
    /**
     * Returns the total time the profile takes to reach the goal.
     *
     * @return The total time the profile takes to reach the goal.
     */
    public double totalTime() {
        double time = 0;
        for (Phase phase : phases) {
            time += phase.time;
        }
        return time;
    }
    /**
     * Returns true if the profile has reached the goal.
     *
     * <p>The profile has reached the goal if the time since the profile started has exceeded the
     * profile's total time.
     *
     * @param time The time since the beginning of the profile.
     * @return True if the profile has reached the goal.
     */
    public boolean isFinished(double time) {
        return time >= totalTime();
    }
}
