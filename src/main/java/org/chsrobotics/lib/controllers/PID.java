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

/**
 * Implementation of a simple Proportional-Integral-Derivative feedback controller.
 *
 * <p>The PID controller is a function with the goal of bringing the difference (or error) between a
 * setpoint (target) and an actual measurement to zero. It does this using feedback control, where
 * the past state(s) of a controlled system are leveraged to find the current output.
 *
 * <p>Since the values of the gains are tuned, they and the output are dimensionless. However, the
 * gains need to be optimized to output what would be sensible values of other units, such as motor
 * controller voltage or duty cycle.
 *
 * <p>The PID controller has three internal components: fittingly, P, I, and D.
 *
 * <p>P, or Proportional, simply consists of multiplication of the error by a constant. This term
 * does the bulk of lifting in the controller and is usually primarily responsible for the bulk of
 * the output. In a theoretical world without static friction and motor input limits, it's enough to
 * get to the setpoint.
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
 * D term should be used when the P controller needed for good performance near the setpoint causes
 * the controller to overshoot the setpoint.
 *
 * <p>When docs in this class refer to "position" or "velocity", "position" refers to the quantity
 * of the thing being controlled, "velocity" to the rate of change of that thing. So it's possible
 * to make a velocity PID controller, or something else with a controlled quantity other than
 * position.
 */
public class PID {

    /** Data class for holding the gains to a PID controller. */
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
    private double lastSetpoint = 0;
    private double integralAccumulation = 0;

    private double setpoint;

    private double velocity = 0;

    private double positionTolerance = 0.02;
    private double velocityTolerance = 0.02;

    /**
     * Constructs a PID with given gains.
     *
     * @param kP The initial proportional gain of the controller.
     * @param kI The initial integral gain of the controller.
     * @param kD The initial derivative gain of the controller.
     * @param initialSetpoint The initial setpoint (or target) of the controller.
     */
    public PID(double kP, double kI, double kD, double initialSetpoint) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        setpoint = initialSetpoint;
        lastSetpoint = initialSetpoint;
    }

    /**
     * Constructs a PID with a given PIDConstants.
     *
     * @param constants The PIDConstants containing the gains for this controller.
     * @param initialSetpoint The initial setpoint (or target) of the controller.
     */
    public PID(PIDConstants constants, double initialSetpoint) {
        this(constants.getkP(), constants.getkI(), constants.getkD(), initialSetpoint);
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
     * Sets the setpoint (target) of the controller.
     *
     * @param value The new target of the controller.
     */
    public void setSetpoint(double value) {
        setpoint = value;
    }

    /**
     * Returns the current setpoint (target) of the controller.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Returns the accumulated past error in the integral term. Not equal to the output of the I
     * term: this is not multiplied by the gain.
     *
     * @return The integral of error with respect to time from the last reset to now.
     */
    public double getIntegralAccumulation() {
        return integralAccumulation;
    }

    /** Resets accumulation of past error in the integral term. */
    public void resetIntegralAccumulation() {
        integralAccumulation = 0;
    }

    /** Resets the previous measurement used for velocity approximation for the derivative term. */
    public void resetPreviousMeasurement() {
        lastMeasurement = 0;
        lastSetpoint = setpoint;
    }

    /** Resets all references to past states in the controller, effectively restarting it. */
    public void reset() {
        resetIntegralAccumulation();
        resetPreviousMeasurement();
    }

    /**
     * Sets the maximum error of both position and velocity allowed for {@code isAtSetpoint()} to
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
     * Returns the maximum allowed position error for {@code isAtSetpoint()} to return true.
     *
     * @return The maximum allowed position error, as a proportion of the setpoint.
     */
    public double getSetpointPositionTolerance() {
        return positionTolerance;
    }

    /**
     * Returns the maximum allowed velocity error for {@code isAtSetpoint()} to return true.
     *
     * @return The maximum allowed absolute velocity per second, as a proportion of the setpoint.
     */
    public double getSetpointVelocityTolerance() {
        return velocityTolerance;
    }

    /**
     * Returns the last reported measurement given to the controller.
     *
     * @return The last reported measurement (0 if none have been given).
     */
    public double getCurrentState() {
        return lastMeasurement;
    }

    /**
     * Returns an output from the controller with a given dt.
     *
     * @param measurement The value of the measured feedback.
     * @param dt The time, in seconds, since the last update of this controller.
     * @return The sum of the P, I, and D terms of the controller.
     */
    public double calculate(double measurement, double dt) {

        integralAccumulation += dt * (setpoint - measurement);

        if (dt == 0) { // sensible way to handle dt of zero
            velocity = 0;
        } else {
            velocity = (((setpoint - measurement) - (lastSetpoint - lastMeasurement)) / dt);
        }

        double p = kP * (setpoint - measurement);
        double i = kI * integralAccumulation;
        double d = kD * velocity;

        lastMeasurement = measurement;
        lastSetpoint = setpoint;

        return p + i + d;
    }

    /**
     * Returns an output from the controller with the default robot loop time.
     *
     * <p>This must be called at a rate of once every robot loop to be consistent.
     *
     * @param measurement The value of the measured feedback.
     * @return The sum of the P, I, and D terms of the controller.
     */
    public double calculate(double measurement) {
        return calculate(measurement, 0.02);
    }

    /**
     * Returns whether the controller has reached the setpoint with minimal velocity.
     *
     * @return Whether the controller is within the minimum position and velocity errors.
     */
    public boolean isAtSetpoint() {
        return (Math.abs(setpoint - lastMeasurement) < Math.abs(positionTolerance * setpoint)
                && Math.abs(velocity) < Math.abs(velocityTolerance * setpoint));
    }
}
