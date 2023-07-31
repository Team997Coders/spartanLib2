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
package org.chsrobotics.lib.hardware.ledStrip;

/**
 * Represents a color in Red-Green-Blue space, with methods for manipulating said color.
 *
 * @param r Red channel value, as a byte [0,255].
 * @param g Green channel value, as a byte [0,255].
 * @param b Blue channel value, as a byte [0,255].
 */
public record RGBColor(int r, int g, int b) {
    /**
     * Interpolates a new color somewhere between this color and another.
     *
     * @param reference Number in [0,1] (inclusive) indicating where to sample the new color.
     *     Smaller values indicate a color closest to this color, larger closer to the other color.
     * @param other Other color to smear towards.
     * @return A new, interpolated color.
     */
    public RGBColor smear(double reference, RGBColor other) {
        double pReference;
        if (reference < 0) pReference = 0;
        else if (reference > 1) pReference = 1;
        else pReference = reference;

        double nR = (this.r * (1 - pReference)) + (other.r * pReference);
        double nG = (this.g * (1 - pReference)) + (other.g * pReference);
        double nB = (this.b * (1 - pReference)) + (other.b * pReference);

        return new RGBColor((int) nR, (int) nG, (int) nB);
    }

    /**
     * Returns a new RGBColor with a modified saturation, or color intensity, from this.
     *
     * @param proportion The amount to multiply each color value by.
     * @return A new RGBColor with modified saturation.
     */
    public RGBColor changeSaturation(double proportion) {
        return new RGBColor(
                (int) (this.r * proportion),
                (int) (this.g * proportion),
                (int) (this.b * proportion));
    }

    /** Default red RGBColor (255,0,0). */
    public static final RGBColor RED = new RGBColor(255, 0, 0);

    /** Default green RGBColor (0,255,0). */
    public static final RGBColor GREEN = new RGBColor(0, 255, 0);

    /** Default blue RGBColor (0,0,255). */
    public static final RGBColor BLUE = new RGBColor(0, 0, 255);

    /** Default orange RGBColor (204,136,0). */
    public static final RGBColor ORANGE = new RGBColor(204, 136, 0);

    /** Default yellow RGBColor (255,255,0). */
    public static final RGBColor YELLOW = new RGBColor(255, 255, 0);

    /** Default indigo RGBColor (111,0,255). */
    public static final RGBColor INDIGO = new RGBColor(111, 0, 255);

    /** Default violet RGBColor (217,25,255). */
    public static final RGBColor VIOLET = new RGBColor(217, 25, 255);

    /** Default white RGBColor (255,255,255). */
    public static final RGBColor WHITE = new RGBColor(255, 255, 255);

    /** Default black RGBColor (0,0,0). */
    public static final RGBColor BLACK = new RGBColor(0, 0, 0);
}
