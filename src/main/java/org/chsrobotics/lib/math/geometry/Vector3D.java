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
package org.chsrobotics.lib.math.geometry;

import java.util.Objects;

/** Represents a 3-dimensional vector in a right-handed system. */
public class Vector3D {
    private final double x;
    private final double y;
    private final double z;

    /**
     * Constructs a 3-dimensional vector.
     *
     * @param x The signed X (i-hat, positive to the horizontal right on a graph) component of the
     *     vector.
     * @param y The signed Y (j-hat, positive up vertically on a graph) component of the vector.
     * @param z The signed Z (k-hat, positive out of the graph) component of the vector.
     */
    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // getX()/getY()/getZ() methods used instead of just x/y/z so that lower-dimensional subclasses
    // can override to 0, to not introduce weird behavior.

    /**
     * Returns the magnitude of the vector as an unsigned (always positive) double.
     *
     * @return The unsigned magnitude.
     */
    public double getMagnitude() {
        return Math.sqrt(
                (this.getX() * this.getX())
                        + (this.getY() * this.getY())
                        + (this.getZ() * this.getZ()));
    }

    /**
     * Adds two vectors.
     *
     * @param other The other vector (order does not matter).
     * @return The result of vector addition.
     */
    public Vector3D add(Vector3D other) {
        return new Vector3D(
                this.getX() + other.getX(), this.getY() + other.getY(), this.getZ() + other.getZ());
    }

    /**
     * Subtracts two vectors.
     *
     * @param other The other vector.
     * @return The result of vector subtraction.
     */
    public Vector3D subtract(Vector3D other) {
        return this.add(other.scalarMultiply(-1));
    }

    /**
     * Performs the dot product operation (sum of the product of parallel component vector signed
     * magnitudes) between two vectors.
     *
     * @param other The other vector (order does not matter).
     * @return The scalar result of the dot product.
     */
    public double dotProduct(Vector3D other) {
        return ((this.getX() * other.getX())
                + (this.getY() * other.getY())
                + (this.getZ() * other.getZ()));
    }

    /**
     * Performs the cross product operation (sum of the perpindicular component vectors) between two
     * vectors.
     *
     * @param other The other vector (order *does* matter).
     * @return The result of the cross product.
     */
    public Vector3D crossProduct(Vector3D other) {
        double xSum = (this.getY() * other.getZ()) - (this.getZ() * other.getY());
        double ySum = (this.getZ() * other.getX()) - (this.getX() * other.getZ());
        double zSum = (this.getX() * other.getY()) - (this.getY() * other.getX());

        return new Vector3D(xSum, ySum, zSum);
    }

    /**
     * Multiplies the vector by a scalar.
     *
     * @param scalar The double scalar to multiply by.
     * @return The result of the multiplication operation.
     */
    public Vector3D scalarMultiply(double scalar) {
        return new Vector3D(this.getX() * scalar, this.getY() * scalar, this.getZ() * scalar);
    }

    /**
     * Divides the vector by a scalar. If {@code scalar} is 0, will return this vector unchanged.
     *
     * @param scalar The double scalar to divide by.
     * @return The result of the division operation.
     */
    public Vector3D scalarDivide(double scalar) {
        if (scalar == 0) return this;
        return this.scalarMultiply(1 / scalar);
    }

    /**
     * Returns the X (i-hat, positive to the horizontal right on a graph) component of the vector.
     *
     * @return The signed X component.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the Y (j-hat, positive up vertically on a graph) component of the vector.
     *
     * @return The signed Y component.
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the Z (k-hat, positive out of the graph) component of the vector.
     *
     * @return The signed Z component.
     */
    public double getZ() {
        return z;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Vector3D) {
            Vector3D rhs = (Vector3D) other;
            return (this.getX() == rhs.getX()
                    && this.getY() == rhs.getY()
                    && this.getZ() == rhs.getZ());
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(this.getX(), this.getY(), this.getZ());
    }

    @Override
    public String toString() {
        return ("Vector: x " + this.getX() + ", y " + this.getY() + ", z " + this.getZ());
    }
}
