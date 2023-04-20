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
import org.chsrobotics.lib.hardware.StalenessWatchable;
import org.chsrobotics.lib.hardware.encoder.AbstractAbsoluteEncoder;
import org.chsrobotics.lib.hardware.encoder.AbstractEncoder;
import org.chsrobotics.lib.hardware.motorController.SpartanSparkMAX.SparkMaxAnalogAbsoluteEncoder.SparkMaxAnalogAbsoluteEncoderConfig;
import org.chsrobotics.lib.hardware.motorController.SpartanSparkMAX.SparkMaxDutyCycleAbsoluteEncoder.SparkMaxDutyCycleAbsoluteEncoderConfig;
import org.chsrobotics.lib.hardware.motorController.SpartanSparkMAX.SparkMaxIncrementalEncoder.SparkMaxIncrementalEncoderConfig;
import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.PeriodicCallbackHandler;

public class SpartanSparkMAX extends AbstractSmartMotorController {
    public static class SparkMaxIncrementalEncoder extends AbstractEncoder {
        public static record SparkMaxIncrementalEncoderConfig(
                boolean inverted, double countsPerRevolution) {
            public SparkMaxIncrementalEncoderConfig setInverted(boolean inverted) {
                return new SparkMaxIncrementalEncoderConfig(inverted, countsPerRevolution);
            }

            public SparkMaxIncrementalEncoderConfig setCountsPerRevolution(
                    double countsPerRevolution) {
                return new SparkMaxIncrementalEncoderConfig(inverted, countsPerRevolution);
            }
        }

        private final SparkMaxIncrementalEncoderConfig config;

        private final RelativeEncoder encoder;

        private int stalenessCount = 0;
        private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

        private SparkMaxIncrementalEncoder(
                SparkMaxIncrementalEncoderConfig config, RelativeEncoder encoder) {
            this.config = config;

            this.encoder = encoder;

            PeriodicCallbackHandler.registerCallback(this::periodic);
        }

        public SparkMaxIncrementalEncoderConfig getConfig() {
            return config;
        }

        @Override
        public double getRawPosition() {
            return encoder.getPosition() * (config.inverted ? -1 : 1);
        }

        @Override
        public double getConvertedPosition() {
            return unitConversion(getRawPosition());
        }

        @Override
        public double getRawVelocity() {
            // native sparkmax differentiation is better than ours
            return encoder.getVelocity() * (config.inverted ? -1 : 1) / 60;
            // convert from RPM to RPS, for consistency
        }

        @Override
        public double getConvertedVelocity() {
            return unitConversion(getRawVelocity());
        }

        private double unitConversion(double in) {
            return in * 2 * Math.PI / config.countsPerRevolution;
        }

        @Override
        public boolean isStale() {
            return (stalenessCount >= stalenessThreshold);
        }

        public void resetStalenessCount() {
            stalenessCount = 0;
        }

        public void setStalenessThreshold(int cycles) {
            stalenessThreshold = cycles;
        }

        private void periodic() {
            if (getRawVelocity() == 0) stalenessCount++;
            else resetStalenessCount();
        }
    }

    public static class SparkMaxAnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {
        public static record SparkMaxAnalogAbsoluteEncoderConfig(
                boolean inverted, double refVoltage, double offset) {
            public SparkMaxAnalogAbsoluteEncoderConfig setInverted(boolean inverted) {
                return new SparkMaxAnalogAbsoluteEncoderConfig(inverted, refVoltage, offset);
            }

            public SparkMaxAnalogAbsoluteEncoderConfig setRefVoltage(double refVoltage) {
                return new SparkMaxAnalogAbsoluteEncoderConfig(inverted, refVoltage, refVoltage);
            }

            public SparkMaxAnalogAbsoluteEncoderConfig setOffset(double offset) {
                return new SparkMaxAnalogAbsoluteEncoderConfig(inverted, offset, offset);
            }
        }

        private final AnalogInput input;

        private final SparkMaxAnalogAbsoluteEncoderConfig config;

        private int stalenessCount = 0;
        private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

        private SparkMaxAnalogAbsoluteEncoder(
                AnalogInput input, SparkMaxAnalogAbsoluteEncoderConfig config) {
            this.input = input;

            this.config = config;

            PeriodicCallbackHandler.registerCallback(this::periodic);
        }

        public SparkMaxAnalogAbsoluteEncoderConfig getConfig() {
            return config;
        }

        @Override
        public double getOffset() {
            return config.offset;
        }

        @Override
        public double getUnoffsetConvertedPosition() {
            return UtilityMath.normalizeAngleRadians(getRawPosition() * 2 * Math.PI);
        }

        @Override
        public double getRawPosition() {
            return (input.getVoltage() / config.refVoltage) * (config.inverted ? -1 : 1);
        }

        @Override
        public boolean isStale() {
            return (stalenessCount >= stalenessThreshold);
        }

        public void resetStalenessCount() {
            stalenessCount = 0;
        }

        public void setStalenessThreshold(int cycles) {
            stalenessThreshold = cycles;
        }

        private void periodic() {
            if (getRawVelocity() == 0) stalenessCount++;
            else resetStalenessCount();
        }
    }

    public static class SparkMaxDutyCycleAbsoluteEncoder extends AbstractAbsoluteEncoder {
        public static record SparkMaxDutyCycleAbsoluteEncoderConfig(
                boolean inverted, double offset) {
            public SparkMaxDutyCycleAbsoluteEncoderConfig setInverted(boolean inverted) {
                return new SparkMaxDutyCycleAbsoluteEncoderConfig(inverted, offset);
            }

            public SparkMaxDutyCycleAbsoluteEncoderConfig setOffset(double offset) {
                return new SparkMaxDutyCycleAbsoluteEncoderConfig(inverted, offset);
            }
        }

        private final SparkMaxAbsoluteEncoder encoder;

        private final SparkMaxDutyCycleAbsoluteEncoderConfig config;

        private int stalenessCount = 0;
        private int stalenessThreshold = StalenessWatchable.defaultStalenessThresholdCycles;

        private SparkMaxDutyCycleAbsoluteEncoder(
                SparkMaxAbsoluteEncoder encoder, SparkMaxDutyCycleAbsoluteEncoderConfig config) {
            this.encoder = encoder;

            this.config = config;

            PeriodicCallbackHandler.registerCallback(this::periodic);
        }

        public SparkMaxDutyCycleAbsoluteEncoderConfig getConfig() {
            return config;
        }

        @Override
        public double getOffset() {
            return config.offset;
        }

        @Override
        public double getUnoffsetConvertedPosition() {
            return UtilityMath.normalizeAngleRadians(2 * Math.PI * getRawPosition());
        }

        @Override
        public double getRawPosition() {
            return encoder.getPosition() * (config.inverted ? -1 : 1);
        }

        // we don't override the getUnconvertedVelocity method because sparkmax natively handles
        // wrap,
        // which is counter the spec of getUnconvertedVelocity()

        @Override
        public double getConvertedVelocity() {
            // sparkmax native differentiation is better than ours
            return encoder.getVelocity() * (config.inverted ? -1 : 1);
        }

        @Override
        public boolean isStale() {
            return (stalenessCount >= stalenessThreshold);
        }

        public void resetStalenessCount() {
            stalenessCount = 0;
        }

        public void setStalenessThreshold(int cycles) {
            stalenessThreshold = cycles;
        }

        private void periodic() {
            if (getRawVelocity() == 0) stalenessCount++;
            else resetStalenessCount();
        }
    }

    public static record SparkMaxConfig(
            int canID,
            MotorType motorType,
            boolean inverted,
            double currentLimitAmps,
            IdleMode initialIdleMode) {
        public static SparkMaxConfig getDefaultBrushless(int canID) {
            return new SparkMaxConfig(canID, MotorType.kBrushless, false, 40, IdleMode.COAST);
        }

        public static SparkMaxConfig getDefaultBrushed(int canID) {
            return new SparkMaxConfig(canID, MotorType.kBrushed, false, 40, IdleMode.COAST);
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

    private int stalenessCount = 0;
    private int stalenessThreshold = 0;

    private double previousBusVoltage = 0;

    public SpartanSparkMAX(SparkMaxConfig config) {
        this.config = config;

        this.sparkMax = new CANSparkMax(config.canID, config.motorType);

        configure();

        PeriodicCallbackHandler.registerCallback(this::periodic);
    }

    public SparkMaxConfig getConfig() {
        return config;
    }

    public SparkMaxIncrementalEncoder getEncoderPortEncoder(
            SparkMaxIncrementalEncoderConfig config) {
        encPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(config, sparkMax.getEncoder());
    }

    public SparkMaxIncrementalEncoder getEncoderPortEncoder(
            Type type, SparkMaxIncrementalEncoderConfig config) {
        encPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(
                config, sparkMax.getEncoder(type, (int) config.countsPerRevolution));
    }

    public SparkMaxIncrementalEncoder getDataPortEncoder(SparkMaxIncrementalEncoderConfig config) {
        dataPortEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, inUsePeriodMS);

        return new SparkMaxIncrementalEncoder(
                config, sparkMax.getAlternateEncoder((int) config.countsPerRevolution));
    }

    public SparkMaxDutyCycleAbsoluteEncoder getDataPortAbsoluteDutyCycleEncoder(
            SparkMaxDutyCycleAbsoluteEncoderConfig config) {
        dutyCycleEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, inUsePeriodMS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, inUsePeriodMS);

        return new SparkMaxDutyCycleAbsoluteEncoder(
                sparkMax.getAbsoluteEncoder(
                        com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle),
                config);
    }

    public SparkMaxAnalogAbsoluteEncoder getDataPortAbsoluteAnalogEncoder(
            boolean inverted, SparkMaxAnalogAbsoluteEncoderConfig config) {
        analogEncoderInUse = true;

        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, inUsePeriodMS);

        return new SparkMaxAnalogAbsoluteEncoder(sparkMax.getAnalog(Mode.kAbsolute), config);
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
            stalenessWatchdogTriggeredLogger.update(isStale());

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
        if (this.setVoltage != volts) {
            sparkMax.setVoltage(volts);
            setVoltage = volts;
        }
    }

    @Override
    public double getSetVoltage() {
        return setVoltage;
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        if (idleMode != this.idleMode) {
            sparkMax.setIdleMode(idleMode.asRev());
            this.idleMode = idleMode;
        }
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

    @Override
    public boolean isStale() {
        return (stalenessCount >= stalenessThreshold);
    }

    public void resetStalenessCount() {
        stalenessCount = 0;
    }

    public void setStalenessThreshold(int cycles) {
        stalenessThreshold = cycles;
    }

    private void periodic(double dtSeconds) {
        // on sticky fault because it's plausible we miss the fault with the way revlib works
        // sticky faults cleared in configure() so this isn't called every cycle
        if (sparkMax.getStickyFault(FaultID.kHasReset)) configure();

        if (getBusVoltage() == previousBusVoltage) stalenessCount++;
        else resetStalenessCount();

        previousBusVoltage = getBusVoltage();
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
