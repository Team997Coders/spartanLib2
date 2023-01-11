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
 * Wraps around an array of RGBColors (or individual pixel color values), intended for combination
 * into full animations.
 */
public class LEDAnimationFrame {
    private final RGBColor[] pixels;

    /**
     * Constructs an LEDAnimationFrame.
     *
     * @param pixelColors Individual pixel colors to form the frame.
     */
    public LEDAnimationFrame(RGBColor... pixelColors) {
        this.pixels = pixelColors;
    }

    /**
     * Returns the number of pixels currently in the animation frame.
     *
     * @return The number of individual colors.
     */
    public int numberOfPixels() {
        return pixels.length;
    }

    /**
     * Gets the pixel color value at an index.
     *
     * @param index The index to sample.
     * @return The color at that index. If not a valid index, returns {@code RGBColor.BLACK}.
     */
    public RGBColor getPixel(int index) {
        if (index < 0 || index >= pixels.length) return RGBColor.BLACK;
        return pixels[index];
    }

    /**
     * Returns a new LEDAnimationFrame based off of this frame, but at a different size.
     *
     * @param newSize The desired size. If less than or equal to 0, returns this frame unchanged.
     * @return A new frame. If the new size is larger than this size, repeats the existing pattern
     *     until the desired size is reached. If the new size is smaller, truncates this frame to
     *     the new size.
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
     * Returns a new LEDAnimationFrame with each color offset (raised or lowered) in index by a
     * given step.
     *
     * @param step The step to offset by.
     * @return A new LEDAnimationFrame.
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
     * Returns a new LEDAnimationFrame of another frame appended to this frame.
     *
     * @param other The frame to append to this.
     * @return A new LEDAnimationFrame.
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
     * Statically constructs and returns a new LEDAnimationFrame of alternating colors. To extend to
     * a desired length, use {@code toNewSize()}.
     *
     * @param numberColorA How many of color A should be put before switching to color B. If less
     *     than or equal to zero, returns an empty frame.
     * @param numberColorB How many of color B to put. If less than or equal to zero, returns and
     *     empty frame.
     * @param colorA The first RGBColor to put.
     * @param colorB The second RGBColor to put.
     * @return A new LEDAnimationFrame.
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
