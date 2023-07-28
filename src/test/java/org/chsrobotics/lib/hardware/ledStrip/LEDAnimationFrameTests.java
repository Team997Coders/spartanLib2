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

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LEDAnimationFrameTests {
    @Test
    public void ledAnimationFrameToNewSizeWorks() {
        LEDAnimationFrame frame =
                new LEDAnimationFrame(RGBColor.WHITE, RGBColor.RED, RGBColor.GREEN);

        assertEquals(new LEDAnimationFrame(RGBColor.WHITE), frame.toNewSize(1));
        assertEquals(new LEDAnimationFrame(RGBColor.WHITE, RGBColor.RED), frame.toNewSize(2));
        assertEquals(frame, frame.toNewSize(3));

        assertEquals(frame.add(new LEDAnimationFrame(RGBColor.WHITE)), frame.toNewSize(4));

        assertEquals(
                frame.add(
                        frame.add(frame.add(new LEDAnimationFrame(RGBColor.WHITE, RGBColor.RED)))),
                frame.toNewSize(11));
    }

    @Test
    public void ledAnimationFrameOffsetWorks() {
        LEDAnimationFrame frame =
                new LEDAnimationFrame(RGBColor.BLACK, RGBColor.BLUE, RGBColor.WHITE);

        assertEquals(
                new LEDAnimationFrame(RGBColor.WHITE, RGBColor.BLACK, RGBColor.BLUE),
                frame.offset(1));

        assertEquals(
                new LEDAnimationFrame(RGBColor.BLUE, RGBColor.WHITE, RGBColor.BLACK),
                frame.offset(-1));

        assertEquals(
                new LEDAnimationFrame(RGBColor.BLUE, RGBColor.WHITE, RGBColor.BLACK),
                frame.offset(2));

        assertEquals(
                new LEDAnimationFrame(RGBColor.WHITE, RGBColor.BLACK, RGBColor.BLUE),
                frame.offset(-2));

        assertEquals(frame.offset(1), frame.offset(4));
        assertEquals(frame.offset(-1), frame.offset(-4));
        assertEquals(frame, frame.offset(3));
        assertEquals(frame, frame.offset(-3));
    }

    @Test
    public void ledAnimationFrameAlternatingWorks() {
        LEDAnimationFrame frame =
                LEDAnimationFrame.alternating(1, 1, RGBColor.WHITE, RGBColor.BLUE);

        assertEquals(new LEDAnimationFrame(RGBColor.WHITE, RGBColor.BLUE), frame);

        frame = LEDAnimationFrame.alternating(5, 1, RGBColor.BLUE, RGBColor.WHITE);

        assertEquals(
                new LEDAnimationFrame(RGBColor.BLUE)
                        .toNewSize(5)
                        .add(new LEDAnimationFrame(RGBColor.WHITE)),
                frame);
    }
}
