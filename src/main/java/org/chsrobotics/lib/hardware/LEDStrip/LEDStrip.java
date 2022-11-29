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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** */
public class LEDStrip {
    private final AddressableLED leds;

    private final AddressableLEDBuffer buffer;

    private final int length;

    private int animationIndex = 0;

    private LEDAnimation animation;

    /**
     * @param PWMChannel
     * @param length
     */
    public LEDStrip(int PWMChannel, int length) {
        leds = new AddressableLED(PWMChannel);
        leds.setLength(length);

        buffer = new AddressableLEDBuffer(length);

        this.length = length;
    }

    /**
     * @param animation
     */
    public void setAnimation(LEDAnimation animation) {
        animationIndex = 0;
        this.animation = animation;
    }

    /** */
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
