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
package org.chsrobotics.lib.math;

import java.util.Map;
import java.util.Map.Entry;
import org.chsrobotics.lib.util.Sampleable;

/**
 * Representation of a standard polynomial function, with restrictions on the upper and lower limits
 * of the domain.
 */
public class PolynomialInterval implements Sampleable<Double> {
    private final Map<Integer, Double> terms;

    private final double min;
    private final double max;

    /**
     * Constructs a PolynomialInterval.
     *
     * @param terms A map of exponent degrees to the coefficient associated with them. For example,
     *     the equation {@code 2x^5 - 3x + 5} would be encoded as a map of {@code (5,2), (1,-3),
     *     (0,5)}. Exponents can be negative, but care should be taken to avoid domain issues (0
     *     should not be part of the allowable values if there are negative exponents).
     * @param min The minimum value to accept as an input to the polynomial function.
     * @param max The maximum value to accept as an input to the polynomial function.
     */
    public PolynomialInterval(Map<Integer, Double> terms, double min, double max) {
        this.terms = terms;

        this.min = min;
        this.max = max;
    }

    @Override
    /** {@inheritDoc} */
    public double getMinReference() {
        return min;
    }

    @Override
    /** {@inheritDoc} */
    public double getMaxReference() {
        return max;
    }

    @Override
    /**
     * Samples the polynomial at a certain value.
     *
     * @param reference The value to sample the polynomial for.
     * @return The output of the polynomial function. If {@code reference} is outside of the
     *     polynomial's domain, returns {@code NaN}.
     */
    public Double sample(double reference) {
        if (isOutOfBounds(reference)) return Double.NaN;

        double sum = 0;

        for (Entry<Integer, Double> entry : terms.entrySet()) {
            sum += Math.pow(reference, entry.getKey()) * entry.getValue();
        }

        return sum;
    }
}
