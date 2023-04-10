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

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.util.datalog.DataLog;
import java.util.ArrayList;
import org.chsrobotics.lib.hardware.encoder.AbstractAbsoluteEncoder;
import org.chsrobotics.lib.hardware.encoder.AbstractIncrementalEncoder;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public class SpartanSparkMAX extends AbstractSmartMotorController {
    public static class SparkMaxIncrementalEncoder extends AbstractIncrementalEncoder {

        private final boolean inverted;
        private final double unitsPerRotation;

        private final RelativeEncoder encoder;

        private SparkMaxIncrementalEncoder(
                boolean inverted, double unitsPerRotation, RelativeEncoder encoder) {
            this.inverted = inverted;
            this.unitsPerRotation = unitsPerRotation;

            this.encoder = encoder;
        }

        @Override
        public double getUnitsPerCount() {
            return unitsPerRotation;
        }

        @Override
        public boolean getInverted() {
            return inverted;
        }

        @Override
        public double getRawCounts() {
            return encoder.getPosition();
        }
    }

    private static class SparkMaxAnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {
        SparkMaxAnalogAbsoluteEncoder(AnalogInput encoder) {}

        @Override
        public double getOffset() {
            throw new UnsupportedOperationException("Unimplemented method 'getOffset'");
        }

        @Override
        public double getUnoffsetConvertedPosition() {
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getUnoffsetConvertedPosition'");
        }

        @Override
        public double getUnoffsetRawPosition() {
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getUnoffsetRawPosition'");
        }

        @Override
        public double getUnitsPerCount() {
            throw new UnsupportedOperationException("Unimplemented method 'getUnitsPerCount'");
        }

        @Override
        public boolean getInverted() {
            throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
        }

        @Override
        public double getRawCounts() {
            throw new UnsupportedOperationException("Unimplemented method 'getRawCounts'");
        }
    }

    private static class SparkMaxDutyCycleAbsoluteEncoder extends AbstractAbsoluteEncoder {
        public SparkMaxDutyCycleAbsoluteEncoder(SparkMaxAbsoluteEncoder encoder) {}

        @Override
        public double getOffset() {
            throw new UnsupportedOperationException("Unimplemented method 'getOffset'");
        }

        @Override
        public double getUnoffsetConvertedPosition() {
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getUnoffsetConvertedPosition'");
        }

        @Override
        public double getUnoffsetRawPosition() {
            throw new UnsupportedOperationException(
                    "Unimplemented method 'getUnoffsetRawPosition'");
        }

        @Override
        public double getUnitsPerCount() {
            throw new UnsupportedOperationException("Unimplemented method 'getUnitsPerCount'");
        }

        @Override
        public boolean getInverted() {
            throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
        }

        @Override
        public double getRawCounts() {
            throw new UnsupportedOperationException("Unimplemented method 'getRawCounts'");
        }
    }

    public static record SparkMaxConfig(
            int canID,
            MotorType motorType,
            boolean inverted,
            double currentLimitAmps,
            IdleMode initialIdleMode) {
        public static SparkMaxConfig getDefaultBrushless() {
            return new SparkMaxConfig(0, MotorType.kBrushless, false, 40, IdleMode.COAST);
        }

        public static SparkMaxConfig getDefaultBrushed() {
            return new SparkMaxConfig(0, MotorType.kBrushed, false, 40, IdleMode.COAST);
        }

        public SparkMaxConfig setCANID(int canID) {
            return new SparkMaxConfig(
                    canID, motorType, inverted, currentLimitAmps, initialIdleMode);
        }

        public SparkMaxConfig setMotorType(MotorType motorType) {
            return new SparkMaxConfig(
                    canID, motorType, inverted, currentLimitAmps, initialIdleMode);
        }

        public SparkMaxConfig setInverted(boolean inverted) {
            return new SparkMaxConfig(
                    canID, motorType, inverted, currentLimitAmps, initialIdleMode);
        }

        public SparkMaxConfig setCurrentLimitAmps(double currentLimitAmps) {
            return new SparkMaxConfig(
                    canID, motorType, inverted, currentLimitAmps, initialIdleMode);
        }

        public SparkMaxConfig setInitialIdleMode(IdleMode initialIdleMode) {
            return new SparkMaxConfig(
                    canID, motorType, inverted, currentLimitAmps, initialIdleMode);
        }
    }

    private final SparkMaxConfig config;

    private final CANSparkMax sparkMax;

    private double setVoltage = 0;
    private IdleMode idleMode;

    private boolean logsConstructed = false;

    private Logger<Boolean> stalenessWatchdogTriggeredLogger;

    private Logger<FaultID[]> faultsLogger;
    private Logger<FaultID[]> stickyFaultsLogger;

    private boolean dutyCycleEncoderInUse = false;
    private boolean analogEncoderInUse = false;
    private boolean encPortEncoderInUse = false;
    private boolean dataPortEncoderInUse = false;

    private final int notInUsePeriodMS = 10000;
    private final int inUsePeriodMS = 20;

    public SpartanSparkMAX(SparkMaxConfig config) {
        this.config = config;

        this.sparkMax = new CANSparkMax(config.canID, config.motorType);

        configure();

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public SparkMaxConfig getConfig() {
        return config;
    }

    public AbstractIncrementalEncoder getEncoderPortEncoder(double unitsPerRotation) {
        encPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(false, unitsPerRotation, sparkMax.getEncoder());
    }

    public AbstractIncrementalEncoder getEncoderPortEncoder(
            Type type, int countsPerRevolution, boolean inverted, double unitsPerRotation) {
        encPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(
                inverted, unitsPerRotation, sparkMax.getEncoder(type, countsPerRevolution));
    }

    public AbstractIncrementalEncoder getDataPortEncoder(
            int countsPerRevolution, boolean inverted, double unitsPerRotation) {
        dataPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(
                inverted, unitsPerRotation, sparkMax.getAlternateEncoder(countsPerRevolution));
    }

    public AbstractAbsoluteEncoder getDataPortAbsoluteDutyCycleEncoder() {
        dutyCycleEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, inUsePeriodMS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, inUsePeriodMS);

        return new SparkMaxDutyCycleAbsoluteEncoder(
                sparkMax.getAbsoluteEncoder(
                        com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle));
    }

    public AbstractAbsoluteEncoder getDataPortAbsoluteAnalogEncoder() {
        analogEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, inUsePeriodMS);

        return new SparkMaxAnalogAbsoluteEncoder(sparkMax.getAnalog(Mode.kAbsolute));
    }

    @Override
    public void autoGenerateLogs(
            DataLog log, String name, String subdirName, boolean publishToNT, boolean recordInLog) {
        if (!logsConstructed) {
            super.autoGenerateLogs(log, name, subdirName, publishToNT, recordInLog);

            faultsLogger =
                    new Logger<>(log, name + "/faults", subdirName, publishToNT, recordInLog);

            stickyFaultsLogger =
                    new Logger<>(log, name + "/stickyFaults", subdirName, publishToNT, recordInLog);

            PeriodicCallbackHandler.registerCallback(this::updateLogs);

            logsConstructed = true;
        }
    }

    private void updateLogs() {
        if (logsConstructed) {
            stalenessWatchdogTriggeredLogger.update(getStalenessWatchdogTriggered());

            ArrayList<FaultID> faults = new ArrayList<>();
            ArrayList<FaultID> stickyFaults = new ArrayList<>();

            for (FaultID fault : FaultID.values()) {
                if (sparkMax.getFault(fault)) faults.add(fault);

                if (sparkMax.getStickyFault(fault)) stickyFaults.add(fault);
            }

            faultsLogger.update(faults.toArray(new FaultID[] {}));

            stickyFaultsLogger.update(faults.toArray(new FaultID[] {}));
        }
    }

    @Override
    public void setVoltage(double volts) {
        sparkMax.setVoltage(volts);
        setVoltage = volts;
    }

    @Override
    public double getSetVoltage() {
        return setVoltage;
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        sparkMax.setIdleMode(idleMode.asRev());
        this.idleMode = idleMode;
    }

    @Override
    public IdleMode getIdleMode() {
        return idleMode;
    }

    @Override
    public double getBusVoltage() {
        return sparkMax.getBusVoltage();
    }

    @Override
    public double getCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public double getMotorTemperature() {
        return sparkMax.getMotorTemperature();
    }

    private void periodic(double dtSeconds) {
        // on sticky fault because it's plausible we miss the fault with the way revlib works
        // sticky faults cleared in configure() so this isn't called every cycle
        if (sparkMax.getStickyFault(FaultID.kHasReset)) configure();
    }

    private void configure() {
        // Configured values which are not burned to flash are lost on power cycle,
        // we need to be able to reapply our settings at any point.

        // We mess around with status frames like this to reduce canbus utilization, which can be a
        // problem. It can be assumed that nobody cares about what data the various
        // sensors hold until they actually get the object for them, and in the same way that they
        // won't stop caring in the middle of runtime (the frames have no way of returning to slow
        // mode in this implementation).

        sparkMax.clearFaults();

        setIdleMode(config.initialIdleMode);

        sparkMax.setInverted(config.inverted);

        sparkMax.setSmartCurrentLimit((int) config.currentLimitAmps);

        sparkMax.setIdleMode(idleMode.asRev());

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, inUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, inUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus2, encPortEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus3, analogEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus4, dataPortEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus5, dutyCycleEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        sparkMax.setPeriodicFramePeriod(
                PeriodicFrame.kStatus6, dutyCycleEncoderInUse ? inUsePeriodMS : notInUsePeriodMS);

        // TODO: timeout of zero may or may not work?
        sparkMax.setCANTimeout(0);
    }
}
