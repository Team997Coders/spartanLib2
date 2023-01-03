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
package org.chsrobotics.lib.math.interpolation;

import java.util.ArrayList;
import org.apache.commons.math3.analysis.function.HarmonicOscillator;
import org.apache.commons.math3.fitting.HarmonicCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.util.Sampleable;

/** TODO doc */
public class HarmonicInterpolation implements Sampleable<Double> {
    private final HarmonicOscillator fit;

    private final double minReference;
    private final double maxReference;

    /**
     * @param maxIterations
     * @param points
     */
    public HarmonicInterpolation(int maxIterations, Vector2D... points) {
        HarmonicCurveFitter fitter = HarmonicCurveFitter.create();
        fitter = fitter.withMaxIterations(maxIterations);

        double tempMax = points[0].getX();
        double tempMin = points[0].getX();

        ArrayList<WeightedObservedPoint> weightedObservedPoints = new ArrayList<>();

        for (int i = 0; i < points.length; i++) {
            if (points[i].getX() > tempMax) tempMax = points[i].getX();
            if (points[i].getX() < tempMin) tempMin = points[i].getX();

            weightedObservedPoints.add(
                    new WeightedObservedPoint(1, points[i].getX(), points[i].getY()));
        }

        double[] harmonicParams = fitter.fit(weightedObservedPoints);

        this.fit = new HarmonicOscillator(harmonicParams[0], harmonicParams[1], harmonicParams[2]);

        this.maxReference = tempMax;
        this.minReference = tempMin;
    }

    @Override
    /**
     * @param reference
     * @return
     */
    public Double sample(double reference) {
        return fit.value(reference);
    }

    @Override
    /** {@inheritDoc} */
    public double getMinReference() {
        return minReference;
    }

    @Override
    /** {@inheritDoc} */
    public double getMaxReference() {
        return maxReference;
    }
}
