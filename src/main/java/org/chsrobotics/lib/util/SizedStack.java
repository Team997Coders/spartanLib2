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
package org.chsrobotics.lib.util;

import java.util.Stack;

/**
 * Extension of a Last-In-First-Out stack which has a cap to the number of stored items. Items are
 * removed from the bottom of the stack to kep within the size limit.
 *
 * @param <T> Data type contained by the stack.
 */
public class SizedStack<T> extends Stack<T> {

    // I have no clue why but javadoc thinks this is public and therefore needs docs
    private final int maxSize;

    /**
     * Constructs a SizedStack.
     *
     * @param maxSize The maximum number of items which this can hold. If less than or equal to 0,
     *     this class will behave identically to a regular {@link Stack}.
     */
    public SizedStack(int maxSize) {
        this.maxSize = maxSize;
    }

    @Override
    /** {@inheritDoc} */
    public T push(T item) {
        super.push(item);

        if (size() > maxSize && maxSize > 0) {
            removeElementAt(0);
        }

        return item;
    }
}
