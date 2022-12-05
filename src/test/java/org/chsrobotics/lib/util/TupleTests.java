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
package org.chsrobotics.lib.util;

import static org.junit.Assert.assertEquals;

import java.util.List;
import org.junit.Test;

public class TupleTests {
    @Test
    public void Tuple2AsListWorksProperly() {
        Tuple2<Double> tuple = new Tuple2<>(1.0, 1.0);

        assertEquals(List.of(1.0, 1.0), tuple.toList());
    }

    @Test
    public void Tuple2OfWorksProperly() {
        Tuple2<Double> tuple = Tuple2.of(1.5, 2.0);

        assertEquals(1.5, tuple.firstValue(), 0);
    }
}
