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
public class LEDAnimationFrame {
    private final RGBColor[] pixels;

    /**
     * @param pixelColors
     */
    public LEDAnimationFrame(RGBColor... pixelColors) {
        this.pixels = pixelColors;
    }

    /**
     * @return
     */
    public int numberOfPixels() {
        return pixels.length;
    }

    /**
     * @param index
     * @return
     */
    public RGBColor getPixel(int index) {
        if (index < 0 || index >= pixels.length) return RGBColor.BLACK;
        return pixels[index];
    }

    /**
     * @param size
     * @return
     */
    public LEDAnimationFrame toNewSize(int newSize) {
        RGBColor[] output = new RGBColor[newSize];
        if (newSize == pixels.length || newSize <= 0 || numberOfPixels() == 0) {
            return this;
        } else if (newSize < pixels.length) {
            for (int i = 0; i < newSize; i++) output[i] = pixels[i];
        } else {
            for (int i = 0; i < newSize; i++) output[i] = pixels[i % pixels.length];
        }
        return new LEDAnimationFrame(output);
    }

    /**
     * @param transform
     * @return
     */
    public LEDAnimationFrame transformColorPlus(RGBColor transform) {
        RGBColor[] output = new RGBColor[pixels.length];

        for (int i = 0; i < pixels.length; i++) output[i] = pixels[i].transformPlus(transform);

        return new LEDAnimationFrame(output);
    }

    /**
     * @param transform
     * @return
     */
    public LEDAnimationFrame transformColorMinus(RGBColor transform) {
        RGBColor[] output = new RGBColor[pixels.length];

        for (int i = 0; i < pixels.length; i++) output[i] = pixels[i].transformMinus(transform);

        return new LEDAnimationFrame(output);
    }

    /**
     * @param step
     * @return
     */
    public LEDAnimationFrame offset(int step) {
        int modStep = step % pixels.length;

        RGBColor[] output = new RGBColor[pixels.length];

        for (int i = 0; i < pixels.length; i++) {
            int newIndex = i + modStep;

            if (newIndex >= pixels.length) newIndex = newIndex - pixels.length;
            else if (newIndex < 0) newIndex = newIndex + pixels.length;

            output[newIndex] = pixels[i];
        }

        return new LEDAnimationFrame(output);
    }

    /**
     * @param other
     * @return
     */
    public LEDAnimationFrame add(LEDAnimationFrame other) {
        RGBColor[] colors = new RGBColor[this.pixels.length + other.pixels.length];

        for (int i = 0; i < this.pixels.length; i++) {
            colors[i] = this.pixels[i];
        }

        for (int i = 0; i < other.pixels.length; i++) {
            colors[i + this.pixels.length] = other.pixels[i];
        }

        return new LEDAnimationFrame(colors);
    }

    /**
     * @param numberColorA
     * @param numberColorB
     * @param colorA
     * @param colorB
     * @return
     */
    public static LEDAnimationFrame alternating(
            int numberColorA, int numberColorB, RGBColor colorA, RGBColor colorB) {
        if (numberColorA <= 0 || numberColorB <= 0) return new LEDAnimationFrame();

        RGBColor[] colors = new RGBColor[numberColorA + numberColorB];

        for (int i = 0; i < numberColorA; i++) {
            colors[i] = colorA;
        }

        for (int i = 0; i < numberColorB; i++) {
            colors[i + numberColorA] = colorB;
        }

        return new LEDAnimationFrame(colors);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof LEDAnimationFrame) {
            LEDAnimationFrame rhs = (LEDAnimationFrame) other;
            if (this.numberOfPixels() == rhs.numberOfPixels()) {
                boolean sameSoFar = true;

                for (int i = 0; i < this.numberOfPixels(); i++) {
                    if (sameSoFar) sameSoFar = this.getPixel(i).equals(rhs.getPixel(i));
                }

                return sameSoFar;
            } else return false;

        } else return false;
    }
}
