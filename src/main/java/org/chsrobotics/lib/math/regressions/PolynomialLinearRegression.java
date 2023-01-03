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

import java.util.Map;
import java.util.Map.Entry;
import org.chsrobotics.lib.math.PolynomialInterval;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.ejml.data.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;

// TODO docs
public class PolynomialLinearRegression {
    public static class PolynomialLinearRegressionResult {
        public final PolynomialInterval polynomial;

        public final double rSquared;

        private PolynomialLinearRegressionResult(
                Map<Integer, Double> exponentsToCoefficients, double rSquared) {
            this.polynomial =
                    new PolynomialInterval(
                            exponentsToCoefficients,
                            Double.NEGATIVE_INFINITY,
                            Double.POSITIVE_INFINITY);

            this.rSquared = rSquared;
        }
    }

    public static PolynomialLinearRegressionResult getPolynomialRegression(
            int order, Vector2D... points) {

        if (order < 1) return null;

        // see: https://mathworld.wolfram.com/LeastSquaresFittingPolynomial.html

        SimpleMatrix leastSquaresMatrix = new SimpleMatrix(points.length, order, double.class);
        SimpleMatrix yMatrix = new SimpleMatrix(points.length, 1, double.class);

        // populate matricies
        for (int i = 0; i < points.length; i++) {
            yMatrix.set(i, 1, points[i].getY());

            for (int u = 0; u <= order; u++) {
                leastSquaresMatrix.set(i, u, Math.pow(points[i].getX(), u));
            }
        }

        SimpleMatrix solVector;
        try {
            solVector =
                    leastSquaresMatrix
                            .transpose()
                            .mult(leastSquaresMatrix)
                            .solve(leastSquaresMatrix.transpose().mult(yMatrix));
        } catch (SingularMatrixException ex) {
            return null;
        }

        Map<Integer, Double> mapForm = Map.of();

        for (int i = 0; i <= order; i++) {
            mapForm.put(i, solVector.get(i, 1));
        }

        double[] ySet = new double[points.length];

        for (int i = 0; i < points.length; i++) {
            ySet[i] = points[i].getY();
        }

        double aMeanY = UtilityMath.arithmeticMean(ySet);

        double sumOfResiduals = 0;

        double sumOfMeanDeviations = 0;

        for (int i = 0; i < points.length; i++) {
            sumOfResiduals += Math.pow(points[i].getY() - pnSample(mapForm, points[i].getX()), 2);
            sumOfMeanDeviations += Math.pow(points[i].getY() - aMeanY, 2);
        }

        if (sumOfMeanDeviations == 0) return null;

        double rSquared = 1 - (sumOfResiduals / sumOfMeanDeviations);

        return new PolynomialLinearRegressionResult(mapForm, rSquared);
    }

    private static double pnSample(Map<Integer, Double> polynomial, double reference) {
        double sum = 0;

        for (Entry<Integer, Double> entry : polynomial.entrySet()) {
            sum += (Math.pow(reference, entry.getKey())) * entry.getValue();
        }

        return sum;
    }
}
