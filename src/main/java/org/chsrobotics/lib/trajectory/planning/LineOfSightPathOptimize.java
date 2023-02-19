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
package org.chsrobotics.lib.trajectory.planning;

import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.lib.math.geometry.Line2D;
import org.chsrobotics.lib.math.geometry.Vector2D;

/**
 * Wraps around a method for optimization of paths (sequences of vector-defined points) in a
 * 2-dimensional configuration space.
 */
public class LineOfSightPathOptimize {

    /**
     * Optimizes the given ordered series of points (represented by the endpoint of vectors).
     *
     * <p>This works via line-of-sight, that is, this seeks the latest point in the path where an
     * unobstructed line in the configuration space can be drawn between the current point and that
     * point. That point is set as the current point, and the process is then repeated until the
     * goal is reached.
     *
     * <p>If the given path is invalid in the given configuration space, this _will_ hang and not
     * finish execution.
     *
     * @param environment The configuration space the points lie in.
     * @param path The ordered series of points to optimize. Must all be valid points in the
     *     configuration space, and each point must have valid line-of-sight to at least its
     *     neighbors in the sequence. The first point in the series is the starting point, the final
     *     is the target.
     * @return An optimized version of the given path. Will be equal or lesser in length, and will
     *     only contain points given in the initial path.
     */
    public static List<Vector2D> lineOfSightOptimize(
            ConfigurationSpace environment, List<Vector2D> path) {
        List<Vector2D> simplifiedList = new ArrayList<>();

        // set the initial working point to the start
        Vector2D workingPoint = path.get(0);

        // while we're not at target yet
        while (!workingPoint.equals(path.get(path.size() - 1))) {

            // add the working point to the target
            if (!simplifiedList.contains(workingPoint)) simplifiedList.add(workingPoint);

            // initially set the best next point to be the current point,
            // any non-malformed path should always at least have connections, if not jumps
            Vector2D candidatePoint = workingPoint;

            // loop through the later nodes in the path
            for (int i = path.indexOf(workingPoint); i < path.size(); i++) {

                // if there's a valid line of sight, that is the new best point
                if (!environment.intersectsObstacle(new Line2D(workingPoint, path.get(i)))) {
                    candidatePoint = path.get(i);
                }
            }

            workingPoint = candidatePoint;
        }

        // have to specially add the final point because loop won't
        simplifiedList.add(path.get(path.size() - 1));

        return simplifiedList;
    }
}
