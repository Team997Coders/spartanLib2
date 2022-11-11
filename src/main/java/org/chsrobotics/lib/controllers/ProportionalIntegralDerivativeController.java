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

import java.util.Objects;
import org.chsrobotics.lib.math.UtilityMath;

/**
 * Implementation of a simple Proportional-Integral-Derivative feedback controller.
 *
 * <p>The PID controller is a function with the goal of bringing the difference (or error) between a
 * setpoint (target) and an actual measurement to zero. It does this using feedback control, where
 * the past state(s) of a controlled system are leveraged to find the current output.
 *
 * <p>Since the values of the gains are tuned, they and the output are dimensionless. However, the
 * gains can be optimized to output what would be sensible values of other units, such as motor
 * controller voltage or duty cycle.
 *
 * <p>The PID controller has three internal components: fittingly, P, I, and D.
 *
 * <p>P, or Proportional, simply consists of multiplication of the error by a constant. This term
 * does the bulk of lifting in the controller and is usually primarily responsible for the bulk of
 * the output. In a theoretical world without static friction and motor input limits, it should be
 * enough to get to the setpoint.
 *
 * <p>I, or Integral, consists of the multiplication of the *sum* of past error by a constant. This
 * past sum should be reset at sensible times to keep it from eccessively accumulating. The Integral
 * term, as it sums up error over time, is able to turn a small amount of constant error the P term
 * can't solve (steady-state error) into enough of a control input to get to the setpoint.
 *
 * <p>D, Derivative and the final term, consists of the multiplication of the *rate of change* of
 * the error by a constant. This is useful to prevent the P term of the controller from entering a
 * series of oscillations where it goes back and forth with high amplitude around the setpoint.
 *
 * <p>Methods for tuning the constants of the PID Controller are a matter of opinion, however, the
 * original author of this doc recommends to start with all constants at 0. Tweak the P term
 * slightly until you arrive at a situation where the P term rapidly gets to the setpoint without
 * overshooting. Any steady-state error that is left can be sparingly corrected with the I term. The
 * D term should be used very carefully as it can cause odd behavior, only save it for times when
 * it's absolutely imperative to reach the setpoint very quickly and an overly aggressive P term is
 * needed.
 */
public class ProportionalIntegralDerivativeController {

    /** Data class for holding the gains to a ProportionalIntegralDerivativeController. */
    public static class PIDConstants {
        private final double kP;
        private final double kI;
        private final double kD;

        /**
         * Constructs a PIDConstants out of provided gains.
         *
         * @param kP The proportional gain.
         * @param kI The integral gain.
         * @param kD The derivative gain.
         */
        public PIDConstants(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        /**
         * Returns the proportional gain.
         *
         * @return The kP of the constants.
         */
        public double getkP() {
            return kP;
        }

        /**
         * Returns the integral gain.
         *
         * @return The kI of the constants.
         */
        public double getkI() {
            return kI;
        }

        /**
         * Returns the derivative gain.
         *
         * @return The kD of the constants.
         */
        public double getkD() {
            return kD;
        }

        @Override
        public boolean equals(Object other) {
            double epsilon = 0.0001;
            if (other instanceof PIDConstants) {
                PIDConstants rhs = (PIDConstants) other;
                return (Math.abs(kP - rhs.kP) < epsilon
                        && Math.abs(kI - rhs.kI) < epsilon
                        && Math.abs(kD - rhs.kD) < epsilon);
            } else {
                return false;
            }
        }

        @Override
        public int hashCode() {
            return Objects.hash(kP, kI, kD);
        }

        @Override
        public String toString() {
            return ("PIDConstants: " + kP + ", " + kI + ", " + kD);
        }
    }

    private double kP;
    private double kI;
    private double kD;

    private double lastMeasurement = 0;
    private double integralAccumulation = 0;

    private double setpoint;

    private double velocity = 0;

    private double positionTolerance = 0.02;
    private double velocityTolerance = 0.02;

    private final boolean isAngular;

    /**
     * Constructs a ProportionalIntegralDerivativeController with given gains.
     *
     * @param kP The initial proportional gain of the controller.
     * @param kI The initial integral gain of the controller.
     * @param kD The initial derivative gain of the controller.
     * @param initialSetpoint The initial setpoint (or target) of the controller.
     * @param isAngular If true, the controller will automatically seek the shortest angle to
     *     travel. However, if using this, values for the {@code measurement} and {@code setpoint}
     *     *must* be in radians.
     */
    public ProportionalIntegralDerivativeController(
            double kP, double kI, double kD, double initialSetpoint, boolean isAngular) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.isAngular = isAngular;
        setpoint = isAngular ? UtilityMath.normalizeAngleRadians(initialSetpoint) : initialSetpoint;
    }

    /**
     * Constructs a ProportionalIntegralDerivativeController with a given PIDConstants.
     *
     * @param constants The PIDConstants containing the gains for this controller.
     * @param initialSetpoint The initial setpoint (or target) of the controller.
     * @param isAngular If true, the controller will automatically seek the shortest angle to
     *     travel. However, if using this, values for the {@code measurement} and {@code setpoint}
     *     *must* be in radians.
     */
    public ProportionalIntegralDerivativeController(
            PIDConstants constants, double initialSetpoint, boolean isAngular) {
        this(constants.getkP(), constants.getkI(), constants.getkD(), initialSetpoint, isAngular);
    }

    /**
     * Sets the gains of the controller using provided gains.
     *
     * @param kP The proportional gain for the controller.
     * @param kI The integral gain for the controller.
     * @param kD The derivative gain for the controller.
     */
    public void setConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Sets the gains of the controller using a provided PIDConstants.
     *
     * @param constants The PIDConstants containing the gains for the controller.
     */
    public void setConstants(PIDConstants constants) {
        this.kP = constants.getkP();
        this.kI = constants.getkI();
        this.kD = constants.getkD();
    }

    /**
     * Returns the current gains of the controller.
     *
     * @return A PIDConstants containing the gains of the controller.
     */
    public PIDConstants getConstants() {
        return new PIDConstants(kP, kI, kD);
    }

    /**
     * Returns the current proportional gain of the controller.
     *
     * @return The current kP.
     */
    public double getkP() {
        return kP;
    }

    /**
     * Sets the proportional gain of the controller.
     *
     * @param kP The new kP for the controller.
     */
    public void setkP(double kP) {
        this.kP = kP;
    }

    /**
     * Returns the current integral gain of the controller.
     *
     * @return The current kI.
     */
    public double getkI() {
        return kI;
    }

    /**
     * Sets the integral gain of the controller.
     *
     * @param kI The new kI for the controller.
     */
    public void setkI(double kI) {
        this.kI = kI;
    }

    /**
     * Returns the current derivative gain of the controller.
     *
     * @return The current kD.
     */
    public double getkD() {
        return kD;
    }

    /**
     * Sets the derivative gain of the controller.
     *
     * @param kD The new kD for the controller.
     */
    public void setkD(double kD) {
        this.kD = kD;
    }

    /**
     * Sets the setpoint (target) of the controller. If this controller is angular, this value
     * *must* be in radians.
     *
     * @param value The new target of the controller.
     */
    public void setSetpoint(double value) {
        setpoint = isAngular ? UtilityMath.normalizeAngleRadians(value) : value;
    }

    /**
     * Returns the current setpoint (target) of the controller.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /** Resets accumulation of past error in the integral term. */
    public void resetIntegralAccumulation() {
        integralAccumulation = 0;
    }

    /** Resets the previous measurement used for velocity approximation for the derivative term. */
    public void resetPreviousMeasurement() {
        lastMeasurement = 0;
    }

    /** Resets all references to past states in the controller, effectively restarting it. */
    public void reset() {
        resetIntegralAccumulation();
        resetPreviousMeasurement();
    }

    /**
     * Returns true if the controller is angular (if will seek the shortest angle to a target).
     *
     * @return Whether the controller will see the setpoints 2pi and 0 as the same.
     */
    public boolean isAngular() {
        return isAngular;
    }

    /**
     * Sets the maximum error of both position and velocity allowed for {@code isFinished()} to
     * return true.
     *
     * @param positionTolerance The maximum allowed position error, as a proportion of the setpoint.
     * @param velocityTolerance The maximum allowed absolute velocity per second, as a proportion of
     *     the setpoint / second.
     */
    public void setSetpointTolerance(double positionTolerance, double velocityTolerance) {
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    /**
     * Returns the maximum allowed position error for {@code isFinished()} to return true.
     *
     * @return The maximum allowed position error, as a proportion of the setpoint.
     */
    public double getSetpointPositionTolerance() {
        return positionTolerance;
    }

    /**
     * Returns the maximum allowed velocity error for {@code isFinished()} to return true.
     *
     * @return The maximum allowed absolute velocity per second, as a proportion of the setpoint.
     */
    public double getSetpointVelocityTolerance() {
        return velocityTolerance;
    }

    /**
     * Returns an output from the controller with a given dt.
     *
     * @param measurement The value of the measured feedback. If this controller is operating in
     *     angular mode, *must* be in radians.
     * @param dt The time, in seconds, since the last update of this controller.
     * @return The sum of the P, I, and D terms of the controller.
     */
    public double calculate(double measurement, double dt) {
        double shortestPathMeasurement =
                UtilityMath.smallestAngleRadiansBetween(measurement, setpoint);

        integralAccumulation += dt * (setpoint - shortestPathMeasurement);
        velocity = ((shortestPathMeasurement - lastMeasurement) / dt);

        double p = kP * (setpoint - shortestPathMeasurement);
        double i = kI * integralAccumulation;
        double d = kD * velocity;

        lastMeasurement = shortestPathMeasurement;

        return p + i + d;
    }

    /**
     * Returns an output from the controller with the default robot loop time.
     *
     * <p>This must be called once every robot loop to be consistent.
     *
     * @param measurement The value of the measured feedback. If this controller is operating in
     *     angular mode, *must* be in radians.
     * @return The sum of the P, I, and D terms of the controller.
     */
    public double calculate(double measurement) {
        return calculate(measurement, 0.02);
    }

    /**
     * Returns whether the controller is at the setpoint and within tolerances specified by {@code
     * setSetpointTolerance()}.
     *
     * @return Whether the controller is within the maximum errors.
     */
    public boolean isAtSetpoint() {
        return (Math.abs(setpoint - lastMeasurement) < (positionTolerance * setpoint)
                && Math.abs(velocity) < (velocityTolerance * setpoint));
    }
}
