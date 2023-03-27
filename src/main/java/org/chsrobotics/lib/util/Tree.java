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
import java.util.List;

/**
 * Implementation of a hierarchial tree data structure.
 *
 * <p>In a tree data structure, each node (except for a designated root) has exactly one parent
 * node, and any number of children nodes.
 *
 * @param <T> Data type to be held by nodes.
 */
public class Tree<T> {

    /**
     * Node part of a tree data structure. This node can have multiple children but only one parent.
     *
     * @param <U> Data type held by this node.
     */
    public static class ParentChildNode<U> {
        private final ParentChildNode<U> parent;
        private final boolean isRoot;

        private U data;

        private final ArrayList<ParentChildNode<U>> children;

        private ParentChildNode(U data, ParentChildNode<U> parent, boolean isRoot) {
            this.parent = parent;
            this.data = data;
            this.isRoot = isRoot;

            children = new ArrayList<>();
        }

        /**
         * Constructs a ParentChildNode.
         *
         * @param data The data to associate with this node.
         * @param parent The parent of this node.
         */
        public ParentChildNode(U data, ParentChildNode<U> parent) {
            this(data, parent, false);
        }

        /**
         * Constructs and returns a new node as a child of this node.
         *
         * @param data Data to associate with the new node.
         * @return A new child of this node.
         */
        public ParentChildNode<U> constructAsChild(U data) {
            ParentChildNode<U> node = new ParentChildNode<>(data, this);

            children.add(node);

            return node;
        }

        /**
         * Returns the data associated with this node.
         *
         * @return An instance of U.
         */
        public U getData() {
            return data;
        }

        /**
         * Sets the new data associated with this node.
         *
         * @param data A new instance of U.
         */
        public void setData(U data) {
            this.data = data;
        }

        /**
         * Returns all children of this node.
         *
         * @return A list containing all children of this node.
         */
        public List<ParentChildNode<U>> getChildren() {
            return children;
        }

        /**
         * Returns the parent of this node.
         *
         * @return The parent of this node. If this node is the root, will return {@code null}.
         */
        public ParentChildNode<U> getParent() {
            return parent;
        }

        /**
         * Returns whether this node is the root of a tree.
         *
         * @return Whether this node is the root.
         */
        public boolean isRoot() {
            return isRoot;
        }
    }

    private final ParentChildNode<T> root;

    /**
     * Constructs a new tree.
     *
     * @param rootData Data to associate with the root of the tree.
     */
    public Tree(T rootData) {
        this.root = new ParentChildNode<>(rootData, null, true);
    }

    /**
     * Returns all nodes that, at some point in their chain of parents, have a certain node as a
     * parent.
     *
     * @param stem The node to find all descendants of.
     * @return All nodes "downstream" of the supplied node, as well as the supplied node. Not in any
     *     particular order.
     */
    public List<ParentChildNode<T>> getBranchNodes(ParentChildNode<T> stem) {
        ArrayList<ParentChildNode<T>> retList = new ArrayList<>();

        retList.add(stem);

        for (ParentChildNode<T> node : stem.getChildren()) {
            retList.addAll(getBranchNodes(node));
        }

        return retList;
    }

    /**
     * Returns all nodes in the tree, including the root.
     *
     * @return All nodes in the tree, as a list in no particular order.
     */
    public List<ParentChildNode<T>> getAllNodes() {
        return getBranchNodes(root);
    }
}
