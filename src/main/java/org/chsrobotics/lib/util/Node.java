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
package org.chsrobotics.lib.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Part of a node-net or node-graph data structure. Supports non-directional connections to other
 * notes with the same data type.
 *
 * @param <T> Data type that can be held by the node.
 */
public class Node<T> {
    private final ArrayList<Node<T>> connections = new ArrayList<>();

    private T data;

    /**
     * Constructs a new node.
     *
     * @param data Data to associate with this node.
     * @param connections Other nodes to initialize this as connected to.
     */
    public Node(T data, List<Node<T>> connections) {
        this.data = data;

        connections.forEach(this::addConnection);
    }

    /**
     * Returns the data associated with this node.
     *
     * @return The data held by the node.
     */
    public T getData() {
        return data;
    }

    /**
     * Sets the data associated with this node to a new value.
     *
     * @param data The data for the node to hold.
     */
    public void setData(T data) {
        this.data = data;
    }

    /**
     * Returns a list of all connected nodes, in no particular order.
     *
     * @return All nodes directly connected to this node.
     */
    public List<Node<T>> getConnections() {
        return connections;
    }

    /**
     * Adds a connection to another node.
     *
     * @param other The node to connect to.
     */
    public void addConnection(Node<T> other) {
        this.addToConnectionArray(other);
        other.addToConnectionArray(this);
    }

    private void addToConnectionArray(Node<T> toAdd) {
        connections.add(toAdd);
    }

    /**
     * Statically connects two nodes of the same type.
     *
     * @param <U> Data type held by the nodes.
     * @param nodeA The first node to connect.
     * @param nodeB The second node to connect.
     */
    public static <U> void addConnection(Node<U> nodeA, Node<U> nodeB) {
        nodeA.addToConnectionArray(nodeB);
        nodeB.addToConnectionArray(nodeA);
    }
}
