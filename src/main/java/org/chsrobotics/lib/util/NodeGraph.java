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
package org.chsrobotics.lib.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

/**
 * Class representing a set of interconnected nodes.
 *
 * @param <T> Data type that can be held by the nodes.
 */
public class NodeGraph<T> {

    /** Class forming a part of the node graph structure. Can hold a single value of type T. */
    public class Node {
        private final int id;

        private final T data;

        private Node(T data, int id) {
            this.data = data;
            this.id = id;
        }

        /**
         * Returns the data stored by this node.
         *
         * @return An instance of T.
         */
        public T getData() {
            return data;
        }
    }

    private int idIndex = 0;

    private HashMap<Integer, Node> nodes = new HashMap<>();

    private HashMap<Integer, ArrayList<Integer>> connectionMap = new HashMap<>();

    /** Constructs a new NodeGraph. */
    public NodeGraph() {}

    /**
     * Creates a new Node as part of this NodeGraph with the given data.
     *
     * @param data The data to initialize the node with.
     * @return A new Node.
     */
    public Node createNode(T data) {
        Node ret = new Node(data, idIndex);

        nodes.put(idIndex, ret);
        connectionMap.put(idIndex, new ArrayList<>());

        idIndex++;

        return ret;
    }

    /**
     * Returns all other nodes connected to the given node.
     *
     * @param node The node to check for connections.
     * @return All nodes connected to the given node.
     */
    public List<Node> getConnectedNodes(Node node) {
        ArrayList<Node> connected = new ArrayList<>();

        for (Integer a : connectionMap.get(node.id)) {
            connected.add(nodes.get(a));
        }

        return connected;
    }

    /**
     * Returns all Nodes in the NodeGraph.
     *
     * @return A Collection of all Nodes in the graph.
     */
    public Collection<Node> getAllNodes() {
        return nodes.values();
    }

    /**
     * Creates a bi-directional connection between two given nodes.
     *
     * @param nodeA The first node to connect.
     * @param nodeB The second node to connect.
     */
    public void connectNodes(Node nodeA, Node nodeB) {
        var newNodeAData = connectionMap.get(nodeA.id);
        newNodeAData.add(nodeB.id);

        connectionMap.put(nodeA.id, newNodeAData);

        var newNodeBData = connectionMap.get(nodeB.id);
        newNodeBData.add(nodeA.id);

        connectionMap.put(nodeB.id, newNodeBData);
    }
}
