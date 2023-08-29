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
package org.chsrobotics.lib.math.filters;

import java.util.ArrayList;
import java.util.List;

/**
 * A filter composed of multiple filters, such that the output of a filter is the input of another,
 * etc.
 */
public class ComposedFilter extends Filter {
    private final ArrayList<Filter> filters;

    private double currentOuput = 0;

    /**
     * Constructs a ComposedFilter.
     *
     * @param filters Filters to compose into this filter. Filters at the start of the list are the
     *     innermost (first) in evaluation, and filters at the end are the outermost (last) in
     *     evaluation.
     */
    public ComposedFilter(List<Filter> filters) {
        this.filters = new ArrayList<>(filters);
    }

    @Override
    /** {@inheritDoc} */
    public double calculate(double value, double dtSeconds) {
        double lastValue = value;

        for (Filter filter : filters) {
            lastValue = filter.calculate(lastValue, dtSeconds);
        }
        currentOuput = lastValue;
        return currentOuput;
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
    public double getCurrentOutput() {
        return currentOuput;
    }
}
