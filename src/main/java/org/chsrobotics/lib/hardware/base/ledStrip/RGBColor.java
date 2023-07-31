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
package org.chsrobotics.lib.hardware.base.ledStrip;

/** Represents a color in Red-Green-Blue space, with methods for manipulating said color. */
public class RGBColor {
    public final int r;
    public final int g;
    public final int b;

    /**
     * Constructs an RGBColor.
     *
     * @param r Red value of the color, in the interval [0,255] (inclusive). If too big or small,
     *     changed to the closest valid value.
     * @param g Green value of the color, in the interval [0,255] (inclusive). If too big or small,
     *     changed to the closest valid value.
     * @param b Blue value of the color, in the interval [0,255] (inclusive). If too big or small,
     *     changed to the closest value value.
     */
    public RGBColor(int r, int g, int b) {
        if (r > 255) this.r = 255;
        else if (r < 0) this.r = 0;
        else this.r = r;

        if (g > 255) this.g = 255;
        else if (g < 0) this.g = 0;
        else this.g = g;

        if (b > 255) this.b = 255;
        else if (b < 0) this.b = 0;
        else this.b = b;
    }

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

    @Override
    public boolean equals(Object other) {
        if (other instanceof RGBColor) {
            RGBColor rhs = (RGBColor) other;
            return (this.r == rhs.r && this.g == rhs.g && this.b == rhs.b);
        } else {
            return false;
        }
    }

    @Override
    public String toString() {
        return "RGBColor (" + r + "," + g + "," + b + ")";
    }

    public static final RGBColor RED = new RGBColor(255, 0, 0);
    public static final RGBColor GREEN = new RGBColor(0, 255, 0);
    public static final RGBColor BLUE = new RGBColor(0, 0, 255);

    public static final RGBColor ORANGE = new RGBColor(204, 136, 0);
    public static final RGBColor YELLOW = new RGBColor(255, 255, 0);
    public static final RGBColor INDIGO = new RGBColor(111, 0, 255);
    public static final RGBColor VIOLET = new RGBColor(217, 25, 255);

    public static final RGBColor WHITE = new RGBColor(255, 255, 255);
    public static final RGBColor BLACK = new RGBColor(0, 0, 0);
}
