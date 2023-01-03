/**
Copyright 2023 FRC Team 997

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
package org.chsrobotics.lib.math.regressions;

import java.util.HashMap;
import java.util.Map;
import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.chsrobotics.lib.math.Polynomial;
import org.chsrobotics.lib.math.geometry.Vector2D;

/**
 * Wraps around a data class for polynomial regression results and static methods for performing
 * these regressions.
 */
public class PolynomialRegression {

    /**
     * A data class packaging the function result of a polynomial regression and the regression's
     * R^2 value.
     */
    public static class PolynomialRegressionResult {

        /** Polynomial function returned as the result of the regression. */
        public final Polynomial polynomial;

        /**
         * R^2 (coefficient of determination) value of the regression, or how much variance is
         * present that isn't explained by the regression function.
         *
         * <p>A value of 0 is the minimum and shows very poor fit between the regression and the
         * data, and a value of 1 is the maximum and an exact fit.
         */
        public final double rSquared;

        private PolynomialRegressionResult(Polynomial polymonial, double rSquared) {
            this.polynomial = polymonial;

            this.rSquared = rSquared;
        }
    }

    /**
     * Performs an Ordinary Least Squares regression upon the points given.
     *
     * @param order Order of the returned polynomial (highest exponent). Must be > 0, or this will
     *     return {@code null}.
     * @param points A series of vectors with endpoints representing data points. The X value is the
     *     explanatory variable, the Y value is the response variable.
     * @return The results of the regression.
     */
    public static PolynomialRegressionResult getPolynomialRegression(
            int order, Vector2D... points) {
        if (order < 1) return null;

        double[] yPopulation = new double[points.length];
        double[][] xPopulation = new double[points.length][order];

        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();

        for (int i = 0; i < points.length; i++) {
            yPopulation[i] = points[i].getY();

            for (int u = 0; u < order; u++) {
                xPopulation[i][u] = Math.pow(points[i].getX(), u + 1);
            }
        }

        regression.newSampleData(yPopulation, xPopulation);

        double[] result = regression.estimateRegressionParameters();

        Map<Integer, Double> polynomialMap = new HashMap<>();

        for (int i = 0; i < result.length; i++) {
            polynomialMap.put(i, result[i]);
        }

        Polynomial polynomial = new Polynomial(polynomialMap);

        return new PolynomialRegressionResult(polynomial, regression.calculateRSquared());
    }

    /**
     * Performs an Ordinary Least Squares Simple Linear regression upon the given data.
     *
     * <p>A Simple Linear regression is a special case of a Polynomial regression.
     *
     * @param points A series of vectors with endpoints representing data points. The X value is the
     *     explanatory variable, the Y value is the response variable.
     * @return The result of the regression.
     */
    public static PolynomialRegressionResult getSimpleLinearRegression(Vector2D... points) {
        return getPolynomialRegression(1, points);
    }
}
