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
public class LEDAnimation {
    private final LEDAnimationFrame[] frames;

    /**
     * @param frames
     */
    public LEDAnimation(LEDAnimationFrame... frames) {
        this.frames = new LEDAnimationFrame[frames.length];

        if (frames.length > 0) {

            int expectedLength = frames[0].numberOfPixels();

            for (int i = 0; i < frames.length; i++)
                this.frames[i] = frames[i].toNewSize(expectedLength);
        }
    }

    /**
     * @return
     */
    public int numberOfFrames() {
        return frames.length;
    }

    /**
     * @param index
     * @return
     */
    public LEDAnimationFrame getFrame(int index) {
        if (frames.length == 0) return new LEDAnimationFrame();

        int pIndex;
        if (index >= frames.length) pIndex = frames.length - 1;
        else if (index < 0) pIndex = 0;
        else pIndex = index;

        return frames[pIndex];
    }

    /**
     * @param length
     * @param step
     * @param colorA
     * @param colorB
     * @return
     */
    public static LEDAnimation gradientCascade(int size, RGBColor colorA, RGBColor colorB) {
        if (size <= 0) return new LEDAnimation();

        LEDAnimationFrame[] lFrames = new LEDAnimationFrame[size];

        RGBColor[] root = new RGBColor[size];

        for (int i = 0; i < size; i++) {
            root[i] = colorA.smear(((double) i) / size, colorB);
        }

        lFrames[0] = new LEDAnimationFrame(root);

        for (int i = 1; i < size; i++) {
            lFrames[i] = lFrames[i - 1].offset(1);
        }

        return new LEDAnimation(lFrames);
    }

    /**
     * @param root
     * @return
     */
    public static LEDAnimation cascading(LEDAnimationFrame root) {
        if (root.numberOfPixels() == 0) return new LEDAnimation();

        LEDAnimationFrame[] lFrames = new LEDAnimationFrame[root.numberOfPixels()];

        lFrames[0] = root;
        for (int i = 1; i < root.numberOfPixels(); i++) {
            lFrames[i] = lFrames[i - 1].offset(1);
        }

        return new LEDAnimation(lFrames);
    }

    /**
     * @param periodCyclesA
     * @param periodCyclesB
     * @param frameA
     * @param frameB
     * @return
     */
    public static LEDAnimation flashing(
            int periodCyclesA,
            int periodCyclesB,
            LEDAnimationFrame frameA,
            LEDAnimationFrame frameB) {
        if (periodCyclesA <= 0
                || periodCyclesB <= 0
                || frameA.numberOfPixels() == 0
                || frameB.numberOfPixels() == 0) return new LEDAnimation();

        LEDAnimationFrame[] lFrames = new LEDAnimationFrame[periodCyclesA + periodCyclesB];

        for (int i = 0; i < periodCyclesA; i++) {
            lFrames[i] = frameA;
        }

        for (int i = 0; i < periodCyclesB; i++) {
            lFrames[i + periodCyclesA] = frameB;
        }

        return new LEDAnimation(lFrames);
    }
}
