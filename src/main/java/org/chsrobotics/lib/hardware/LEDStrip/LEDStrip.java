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
package org.chsrobotics.lib.hardware.LEDStrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Convenience wrapper around an addressable LED strip (WS2812 or 2811). */
public class LEDStrip {
    private final AddressableLED leds;

    private final AddressableLEDBuffer buffer;

    private final int length;

    private int animationIndex = 0;

    private LEDAnimation animation = new LEDAnimation();

    /**
     * Constructs a new LEDStrip.
     *
     * @param PWMChannel PWM channel to use to send data to the strip.
     * @param length The number of pixels in the strip.
     */
    public LEDStrip(int PWMChannel, int length) {
        leds = new AddressableLED(PWMChannel);
        leds.setLength(length);

        buffer = new AddressableLEDBuffer(length);

        this.length = length;
    }

    /**
     * Sets the currently displayed LEDAnimation.
     *
     * @param animation The LEDAnimation to play through.
     */
    public void setAnimation(LEDAnimation animation) {
        animationIndex = 0;
        this.animation = animation;
    }

    /**
     * Sets the currently displayed LEDAnimation to an animation consisting of only a given frame.
     *
     * @param frame The LEDAnimationFrame to display.
     */
    public void setFrame(LEDAnimationFrame frame) {
        animationIndex = 0;
        this.animation = new LEDAnimation(frame);
    }

    /** Advances to the next step of the animation. */
    public void update() {
        if (animation.numberOfFrames() != 0) {
            LEDAnimationFrame frame = animation.getFrame(animationIndex).toNewSize(length);

            for (int i = 0; i < frame.numberOfPixels(); i++) {
                RGBColor pixel = frame.getPixel(i);
                buffer.setRGB(i, pixel.r, pixel.g, pixel.b);
            }

            if (animationIndex == animation.numberOfFrames() - 1) animationIndex = 0;
            else animationIndex++;
        }
    }
}
