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
package org.chsrobotics.lib.hardware.LEDStrip;

/** */
public class RGBColor {
    public final int r;
    public final int g;
    public final int b;

    /**
     * @param r
     * @param g
     * @param b
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
     * @param reference
     * @param other
     * @return
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
     * @param other
     * @return
     */
    public RGBColor transformPlus(RGBColor other) {
        return new RGBColor(this.r + other.r, this.g + other.g, this.b + other.b);
    }

    /**
     * @param other
     * @return
     */
    public RGBColor transformMinus(RGBColor other) {
        return new RGBColor(this.r - other.r, this.g - other.g, this.b - other.b);
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

    public static final RGBColor WHITE = new RGBColor(255, 255, 255);
    public static final RGBColor BLACK = new RGBColor(0, 0, 0);
}
