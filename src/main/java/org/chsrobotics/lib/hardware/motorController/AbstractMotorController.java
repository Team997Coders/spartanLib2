/**
Copyright 2023 FRC Team 997

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
package org.chsrobotics.lib.hardware.motorController;

import org.chsrobotics.lib.telemetry.IntrinsicLoggable;

public abstract class AbstractMotorController implements IntrinsicLoggable {
    public static enum IdleMode {
        COAST,
        BRAKE;

        public com.revrobotics.CANSparkMax.IdleMode asRev() {
            if (this == IdleMode.BRAKE) return com.revrobotics.CANSparkMax.IdleMode.kBrake;
            else return com.revrobotics.CANSparkMax.IdleMode.kCoast;
        }
    }

    public abstract void setVoltage(double volts);

    public abstract double getSetVoltage();

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract IdleMode getIdleMode();
}
