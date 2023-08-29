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

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Wraps around an array of LEDAnimationFrames, intended for playing back in sequence. */
public class SimpleLEDAnimation implements LEDAnimation {
    private final List<LEDAnimationFrame> frames;

    private final int expectedLength;

    private int animationIndex = 0;

    /**
     * Constructs a new SimpleLEDAnimation.
     *
     * @param frames Frames to add to the animation. If a first frame is present, all other frames
     *     will be reshaped to match it in length.
     */
    public SimpleLEDAnimation(List<LEDAnimationFrame> frames) {
        this.frames = new ArrayList<>();

        if (frames.size() > 0) {

            int expectedLength = frames.get(0).numberOfPixels();

            for (int i = 0; i < frames.size(); i++)
                this.frames.add(frames.get(i).toNewSize(expectedLength));

            this.expectedLength = expectedLength;
        } else expectedLength = 0;
    }

    /**
     * Returns the number of frames the animation consists of.
     *
     * @return The number of frames present.
     */
    public int numberOfFrames() {
        return frames.size();
    }

    @Override
    /** {@inheritDoc} */
    public int numberOfPixelsPerFrame() {
        return expectedLength;
    }

    @Override
    /** {@inheritDoc} */
    public LEDAnimationFrame getNextFrame() {
        if (numberOfFrames() != 0) {
            int preIndex = animationIndex;

            if (animationIndex == numberOfFrames() - 1) animationIndex = 0;
            else animationIndex++;

            return getFrame(preIndex).toNewSize(expectedLength);

        } else return new LEDAnimationFrame();
    }

    /**
     * Returns the LEDAnimationFrame at an index.
     *
     * @param index The index to sample.
     * @return The LEDAnimationFrame at that index.
     */
    public LEDAnimationFrame getFrame(int index) {
        if (frames.size() == 0) return new LEDAnimationFrame();

        int pIndex;
        if (index >= frames.size()) pIndex = frames.size() - 1;
        else if (index < 0) pIndex = 0;
        else pIndex = index;

        return frames.get(pIndex);
    }

    /**
     * Statically constructs and returns a new SimpleLEDAnimation of a gradient between two colors
     * which offsets itself by one step each cycle.
     *
     * @param size Number of pixels in the LED strip. If 0 or less, returns an empty animation.
     * @param colorA First color of the gradient.
     * @param colorB Second color of the gradient.
     * @return A new LEDAnimation.
     */
    public static SimpleLEDAnimation gradientCascade(int size, RGBColor colorA, RGBColor colorB) {
        if (size <= 0) return new SimpleLEDAnimation(List.of());

        ArrayList<RGBColor> root = new ArrayList<>();

        for (int i = 0; i < size - 1; i++) {
            root.add(colorA.smear(((double) i) / (size - 1), colorB));
        }

        root.add(colorB);

        return cascading(new LEDAnimationFrame(root));
    }

    /**
     * Statically constructs and returns a new SimpleLEDAnimation which offsets itself from an
     * initial frame by one step each cycle.
     *
     * @param root The initial frame of the animation.
     * @return A new LEDAnimation.
     */
    public static SimpleLEDAnimation cascading(LEDAnimationFrame root) {
        if (root.numberOfPixels() == 0) return new SimpleLEDAnimation(List.of());

        ArrayList<LEDAnimationFrame> lFrames = new ArrayList<>();

        lFrames.add(root);

        for (int i = 1; i < root.numberOfPixels(); i++) {
            lFrames.add(lFrames.get(lFrames.size() - 1).offset(1));
        }

        return new SimpleLEDAnimation(lFrames);
    }

    /**
     * Statically constructs and returns a new SimpleLEDAnimation which periodically cycles between
     * two frames.
     *
     * @param periodCyclesA How many consecutive cycles frameA should be shown for.
     * @param periodCyclesB How many consecutive cycles frameB should be shown for.
     * @param frameA The first frame to periodically display.
     * @param frameB The second frame to periodically display.
     * @return A new LEDAnimation.
     */
    public static SimpleLEDAnimation flashing(
            int periodCyclesA,
            int periodCyclesB,
            LEDAnimationFrame frameA,
            LEDAnimationFrame frameB) {
        if (periodCyclesA <= 0
                || periodCyclesB <= 0
                || frameA.numberOfPixels() == 0
                || frameB.numberOfPixels() == 0) return new SimpleLEDAnimation(List.of());

        ArrayList<LEDAnimationFrame> lFrames = new ArrayList<>();

        for (int i = 0; i < periodCyclesA; i++) {
            lFrames.add(frameA);
        }

        for (int i = 0; i < periodCyclesB; i++) {
            lFrames.add(frameB);
        }

        return new SimpleLEDAnimation(lFrames);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof SimpleLEDAnimation) {
            SimpleLEDAnimation rhs = (SimpleLEDAnimation) other;
            if (this.numberOfFrames() == rhs.numberOfFrames()) {
                boolean sameSoFar = true;

                for (int i = 0; i < this.numberOfFrames(); i++) {
                    if (sameSoFar) sameSoFar = this.getFrame(i).equals(rhs.getFrame(i));
                }

                return sameSoFar;
            } else return false;

        } else return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(frames);
    }
}
