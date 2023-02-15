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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.util.NodeGraph;

/**
 * Implementation of Edsger Dijkstra's algorithm for cost-optimal graph traversal.
 *
 * <p>This algorithm operates in O(E log(V)) time, where E is the number of connections between
 * nodes, and V is the number of nodes.
 *
 * <p>This algorithm is suited well for small groups of nodes, where it produces an optimal solution
 * quickly. However, as the problem-space grows more complex, it may begin to make more sense to use
 * another pathfinding algorithm like RRT.
 *
 * <p>The algorithm also requires a pre-built map of nodes and connections, which can require
 * complex graph structures for some problems.
 */
public class Dijkstra {

    /**
     * Interface implementing a cost function for Dijkstra's algorithm, which returns the cost of
     * traversing from a node holding a certain value to another.
     */
    public interface CostFunction<T> {
        double evaluate(T valueA, T valueB);
    }

    private static class VectorCostFunction implements CostFunction<Vector2D> {
        @Override
        public double evaluate(Vector2D valueA, Vector2D valueB) {
            return valueA.subtract(valueB).getMagnitude();
        }
    }

    /**
     * Generates a cost-optimal path through the given connected graph nodes from a source node to a
     * target.
     *
     * @param <T> The data type of the nodes.
     * @param nodes The set of connected nodes to find a path through.
     * @param source The source, or starting, node of the path. Should be connected to the other
     *     nodes, but should not be a member of the object already passed.
     * @param target The target node of the path. Should be connected to the other nodes, but should
     *     not be a member of the object already passed.
     * @param costFunction A function to determine the cost of traveling from one node to another,
     *     based on the contained data.
     * @return An ordered list of the data type, representing the optimal path through the nodes
     *     (including source and target nodes).
     */
    public static <T> List<T> generatePath(
            NodeGraph<T> nodes,
            NodeGraph<T>.Node source,
            NodeGraph<T>.Node target,
            CostFunction<T> costFunction) {

        Map<NodeGraph<T>.Node, NodeGraph<T>.Node> previousNodes = new HashMap<>();
        Map<NodeGraph<T>.Node, Double> costs = new HashMap<>();

        List<NodeGraph<T>.Node> unexploredNodes = new ArrayList<>();

        // populate initial data
        for (NodeGraph<T>.Node node : nodes.getAllNodes()) {
            previousNodes.put(node, null);
            costs.put(node, Double.POSITIVE_INFINITY);

            unexploredNodes.add(node);
        }

        unexploredNodes.add(source);
        unexploredNodes.add(target);

        previousNodes.put(source, null);
        costs.put(source, 0.0);

        costs.put(target, Double.POSITIVE_INFINITY);

        while (unexploredNodes.size() > 0) {
            NodeGraph<T>.Node currentNode =
                    unexploredNodes.get(0); // needed to keep compiler from screaming

            // sets the working node to unexplored node with smallest cost

            // on first iteration, all nodes but start have infinite cost, making working node the
            // start node
            for (NodeGraph<T>.Node testNode : unexploredNodes) {
                if (costs.get(testNode) < costs.get(currentNode)) {
                    currentNode = testNode;
                }
            }

            // if a path exists to the target and it is shorter than all other active paths, break
            // the loop, we're done
            if (currentNode.equals(target)) break;

            unexploredNodes.remove(currentNode);

            // loop through all neighbors of the point that have not been explored
            for (NodeGraph<T>.Node neighbor : nodes.getConnectedNodes(currentNode)) {
                if (unexploredNodes.contains(neighbor)) {
                    double altCost =
                            costs.get(currentNode)
                                    + costFunction.evaluate(
                                            currentNode.getData(), neighbor.getData());

                    // if a shorter path to the neighbor exists through the current node, compared
                    // with the shortest already discovered path, replace that path with this
                    if (altCost < costs.get(neighbor)) {
                        costs.put(neighbor, altCost);
                        previousNodes.put(neighbor, currentNode);
                    }
                }
            }
        }

        NodeGraph<T>.Node currentNode = target;
        ArrayList<T> sequence = new ArrayList<>();

        // step backwards from the target, using stored previous connections, to the start (where
        // the previous node == null)
        while (currentNode != null) {
            sequence.add(0, currentNode.getData());
            currentNode = previousNodes.get(currentNode);
        }

        return sequence;
    }

    /**
     * Generates a Euclidian-distance-optimal path through a series of nodes containing vectors.
     *
     * @param nodes The set of connected nodes to find a path through.
     * @param source The source, or starting, node of the path. Should be connected to the other
     *     nodes, but should not be a member of the object already passed.
     * @param target The target node of the path. Should be connected to the other nodes, but should
     *     not be a member of the object already passed.
     * @return An ordered list of vectors, representing the shortest path, through the nodes, from
     *     startpoint to endpoint. Contains startpoint and endpoint.
     */
    public static List<Vector2D> generateSpatialPath(
            NodeGraph<Vector2D> nodes,
            NodeGraph<Vector2D>.Node source,
            NodeGraph<Vector2D>.Node target) {

        return generatePath(nodes, source, target, new VectorCostFunction());
    }

    /**
     * Returns the cost to navigate through a given path.
     *
     * @param <T> The data type of the path.
     * @param path A list of ordered data to find the total cost of.
     * @param costFunction A function which returns the cost to travel between two objects.
     * @return The cost to navigate through a given path.
     */
    public static <T> double getTotalCost(List<T> path, CostFunction<T> costFunction) {
        double cost = 0;

        for (int i = 1; i < path.size(); i++) {
            cost += costFunction.evaluate(path.get(i), path.get(i - 1));
        }

        return cost;
    }

    /**
     * Returns the cost to navigate through a given path in 2-dimensional Euclidean space.
     *
     * @param path A list of ordered data to find the total cost of.
     * @return The cost to navigate through a given path.
     */
    public static double getTotalSpatialCost(List<Vector2D> path) {
        return getTotalCost(path, new VectorCostFunction());
    }
}
