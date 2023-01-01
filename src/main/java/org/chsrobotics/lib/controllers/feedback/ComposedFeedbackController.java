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
package org.chsrobotics.lib.controllers.feedback;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.chsrobotics.lib.math.filters.Filter;

/**
 * A controller made of a sum of filters, each multiplied by a constant gain. Instead of the input
 * value, the error between the setpoint and input is what is given to the filters.
 *
 * <p>For example, this can easily be turned into a simple PID controller by constructing as
 * follows:
 *
 * <p>{@code new ComposedController(Map.of(new NullFilter(), kP, new IntegratingFilter(window), kI,
 * new DifferentiatingFilter(), kD))}
 */
public class ComposedFeedbackController implements FeedbackController {
    private final List<Filter> filters;

    private double setpoint = 0;

    private double currentValue = 0;

    private double velocity = 0;

    private double lastMeasurement = 0;
    private double lastSetpoint = 0;

    private double positionTolerance = FeedbackController.defaultPositionErrorToleranceProportion;
    private double velocityTolerance = FeedbackController.defaultVelocityErrorToleranceProportion;

    /**
     * Constructs a ComposedController.
     *
     * @param filters A list of filters to combine into the controller.
     */
    public ComposedFeedbackController(List<Filter> filters) {
        this.filters = filters;
    }

    /**
     * Constructs a ComposedController.
     *
     * @param values A map of filters to their gains.
     */
    public ComposedFeedbackController(Map<Filter, Double> values) {
        ArrayList<Filter> filters = new ArrayList<>();

        for (Filter filter : values.keySet()) {
            filters.add(Filter.scalarMultiply(filter, values.get(filter)));
        }

        this.filters = filters;
    }

    @Override
    /** {@inheritDoc} */
    public void reset() {
        for (Filter filter : filters) {
            filter.reset();
        }
    }

    @Override
    /** {@inheritDoc} */
    public void setSetpoint(double value) {
        setpoint = value;
    }

    @Override
    /** {@inheritDoc} */
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double measurement) {
        return calculate(measurement, 0.02);
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double measurement, double dtSeconds) {
        double error = setpoint - measurement;

        currentValue = 0;

        if (dtSeconds == 0) {
            velocity = 0;
        } else {
            velocity = ((setpoint - measurement) - (lastSetpoint - lastMeasurement)) / dtSeconds;
        }

        for (Filter filter : filters) {
            currentValue += filter.calculate(error, dtSeconds);
        }

        lastMeasurement = measurement;
        lastSetpoint = setpoint;

        return currentValue;
    }

    @Override
    /** {@inheritDoc} */
    public double getCurrentValue() {
        return currentValue;
    }

    @Override
    /** {@inheritDoc} */
    public void setSetpointTolerance(
            double positionErrorProportion, double velocityErrorProportion) {
        positionTolerance = positionErrorProportion;
        velocityTolerance = velocityErrorProportion;
    }

    @Override
    /** {@inheritDoc} */
    public boolean atSetpoint() {
        return (Math.abs(setpoint - lastMeasurement) < Math.abs(positionTolerance * setpoint)
                && Math.abs(velocity) < Math.abs(velocityTolerance * setpoint));
    }
}
