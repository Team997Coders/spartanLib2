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
package org.chsrobotics.lib.telemetry;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.datalog.DataLog;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Utility to streamline publishing data to NetworkTables and saving it to an on-robot log file.
 *
 * <p>Parameterized to the type of data to publish/log.
 *
 * <p>If the data isn't numerical (Double, Int, Long, etc), Boolean, String, or an Array of one of
 * those types, this will instead publish and log the result of its {@code .toString()} method.
 *
 * <p>This can generally be used in two ways: construction with a lambda of the logged value, or
 * passing the new value with every call of {@code update()}.
 *
 * <p>Values are only flushed to NetworkTables and built-in logs when {@code update()} is called.
 *
 * @param <T> The data type of the log entry.
 */
public class Logger<T> {

    /**
     * Factory class to simplify creating many similar logs at once.
     *
     * @param <U> Data type of the loggers to create.
     */
    public static class LoggerFactory<U> {
        private final DataLog log;
        private final String subdirName;
        private final boolean publishToNT;
        private final boolean recordInLog;

        /**
         * Constructs a LoggerFactory.
         *
         * <p>All Loggers constructed with this factory will share these settings.
         *
         * @param log The DataLog to log values inside of, most likely from {@code
         *     HighLevelLogger.getInstance().getLog()} or whatever log is being used program-wide.
         * @param subdirName The string name of the existing or new NetworkTables sub-table to write
         *     to.
         * @param publishToNT Whether this should push logged values to NetworkTables.
         * @param recordInLog Whether this should store logged values in an on-robot log file.
         */
        @SuppressFBWarnings("EI_EXPOSE_REP2")
        public LoggerFactory(
                DataLog log, String subdirName, boolean publishToNT, boolean recordInLog) {
            this.log = log;
            this.subdirName = subdirName;
            this.publishToNT = publishToNT;
            this.recordInLog = recordInLog;
        }

        /**
         * Constructs a Logger using {@code HighLevelLogger.getLog()}, publishing to NT and logging.
         *
         * @param subdirName The string name of the existing or new NetworkTables sub-table to write
         *     to.
         */
        public LoggerFactory(String subdirName) {
            this(HighLevelLogger.getInstance().getLog(), subdirName, true, true);
        }

        /**
         * Constructs and returns a new logger with the parameters given above.
         *
         * @param key A string identifier for the logged field.
         * @return A new Logger.
         */
        public Logger<U> getLogger(String key) {
            return new Logger<>(log, key, subdirName, publishToNT, recordInLog);
        }

        /**
         * Constructs and returns a new logger with the parameters given above.
         *
         * @param key A string identifier for the logged field.
         * @param lambda Lambda (of logged type) to use as initial data source.
         * @return A new Logger.
         */
        public Logger<U> getLogger(String key, Supplier<U> lambda) {
            return new Logger<>(lambda, log, key, key, publishToNT, recordInLog);
        }
    }

    private final DataLog log;
    private boolean publishToNT;
    private boolean recordInLog;

    private final String logEntryIdentifier;

    private final Topic ntTopic;

    private GenericPublisher publisher = null;

    private int logHandle = 0; // not a valid handle

    private Object prevVal = null;

    private Supplier<T> lambda;

    /**
     * Constructs a Logger using a provided DataLog, with the option of whether to publish to NT and
     * the log.
     *
     * @param lambda Lambda (of logged type) to use as initial data source.
     * @param log The DataLog to log values inside of, most likely from {@code
     *     HighLevelLogger.getInstance.getLog()} or whatever log is being used program-wide.
     * @param key A string identifier for the logged field.
     * @param subdirName The string name of the existing or new NetworkTables sub-table to write to.
     * @param publishToNT Whether this should push logged values to NetworkTables.
     * @param recordInLog Whether this should store logged values in an on-robot log file.
     */
    @SuppressFBWarnings("EI_EXPOSE_REP2")
    public Logger(
            Supplier<T> lambda,
            DataLog log,
            String key,
            String subdirName,
            boolean publishToNT,
            boolean recordInLog) {
        this.log = log;
        this.publishToNT = publishToNT;
        this.recordInLog = recordInLog;

        logEntryIdentifier = subdirName + "_" + key;

        ntTopic = NetworkTableInstance.getDefault().getTable(subdirName).getTopic(key);

        this.lambda = lambda;
    }

    /**
     * Constructs a Logger using a provided DataLog, with the option of whether to publish to NT and
     * the log.
     *
     * @param log The DataLog to log values inside of, most likely from {@code
     *     HighLevelLogger.getInstance.getLog()} or whatever log is being used program-wide.
     * @param key A string identifier for the logged field.
     * @param subdirName The string name of the existing or new NetworkTables sub-table to write to.
     * @param publishToNT Whether this should push logged values to NetworkTables.
     * @param recordInLog Whether this should store logged values in an on-robot log file.
     */
    public Logger(
            DataLog log, String key, String subdirName, boolean publishToNT, boolean recordInLog) {
        this(() -> null, log, key, subdirName, publishToNT, recordInLog);
    }

    /**
     * Constructs a Logger using {@code HighLevelLogger.getLog()}, publishing to NT and logging.
     *
     * @param lambda Lambda (of logged type) to use as initial data source.
     * @param key A string identifier for the logged field.
     * @param subdirName The string name of the existing or new NetworkTables sub-table to write to.
     */
    public Logger(Supplier<T> lambda, String key, String subdirName) {
        this(HighLevelLogger.getInstance().getLog(), key, subdirName, true, true);
    }

    /**
     * Constructs a Logger using {@code HighLevelLogger.getLog()}, publishing to NT and logging.
     *
     * @param key A string identifier for the logged field.
     * @param subdirName The string name of the existing or new NetworkTables sub-table to write to.
     */
    public Logger(String key, String subdirName) {
        this(() -> null, key, subdirName);
    }

    /**
     * Returns whether this is publishing values to NetworkTables.
     *
     * @return True new values are pushed to NetworkTables.
     */
    public boolean isPublishingToNT() {
        return publishToNT;
    }

    /** Values will be pushed to NetworkTables. */
    public void startPublishingToNT() {
        publishToNT = true;
    }

    /** Values will not be pushed to NetworkTables. */
    public void stopPublishingToNT() {
        publishToNT = false;
    }

    /**
     * Returns whether this is recording values to the on-robot log.
     *
     * @return True if values are being recorded to the on board data log.
     */
    public boolean isRecordingToLog() {
        return recordInLog;
    }

    /** Values will be recorded to the on board data log. */
    public void startRecordingToLog() {
        recordInLog = true;
    }

    /** Values will not be recorded to the on board data log. */
    public void stopRecordingToLog() {
        recordInLog = false;
    }

    /**
     * Updates data sinks with the current internal value.
     *
     * <p>If this was not constructed with a lambda and other {@code update()} methods have not been
     * invoked, is a non-op.
     */
    public void update() {
        Object value = lambda.get();

        if (!Objects.equals(value, prevVal)) {
            NetworkTableType dataType =
                    NetworkTableType.getFromString(NetworkTableType.getStringFromObject(value));

            if (value != null) {
                // special-case logging to match AdvantageScope expected formats
                if (value instanceof Pose2d) {
                    Pose2d castValue = (Pose2d) value;

                    value =
                            new Double[] {
                                castValue.getX(),
                                castValue.getY(),
                                castValue.getRotation().getRadians()
                            };

                    dataType = NetworkTableType.kDoubleArray;
                }

                if (value instanceof Pose3d) {
                    Pose3d castValue = (Pose3d) value;

                    value =
                            new Double[] {
                                castValue.getX(),
                                castValue.getY(),
                                castValue.getZ(),
                                castValue.getRotation().getAngle(), // w
                                castValue.getRotation().getX(),
                                castValue.getRotation().getY(),
                                castValue.getRotation().getZ()
                            };

                    dataType = NetworkTableType.kDoubleArray;
                }

                if (value instanceof Rotation2d) {
                    Rotation2d castValue = (Rotation2d) value;

                    value = castValue.getRadians();

                    dataType = NetworkTableType.kDouble;
                }

                if (value instanceof Rotation3d) {
                    Rotation3d castValue = (Rotation3d) value;

                    value =
                            new Double[] {
                                castValue.getAngle(), // w
                                castValue.getX(),
                                castValue.getY(),
                                castValue.getZ()
                            };

                    dataType = NetworkTableType.kDoubleArray;
                }

                if (dataType == NetworkTableType.kUnassigned) {
                    value = value.toString();
                }

                // don't want to accidentially lock ourselves out of
                // floating-point numbers if first logged value is integer,
                // so just cast all numerical types to double
                if (dataType == NetworkTableType.kFloat || dataType == NetworkTableType.kInteger) {
                    dataType = NetworkTableType.kDouble;
                }
                if (dataType == NetworkTableType.kFloatArray
                        || dataType == NetworkTableType.kIntegerArray) {
                    dataType = NetworkTableType.kDoubleArray;
                }

                if (publisher == null) {
                    publisher = ntTopic.genericPublish(NetworkTableType.getStringFromObject(value));
                }

                publisher.setValue(value);

                if (logHandle == 0) {
                    logHandle = log.start(logEntryIdentifier, dataType.getValueStr());
                }

                switch (dataType) {
                    case kBoolean:
                        log.appendBoolean(logHandle, (boolean) value, 0);
                        break;
                    case kDouble:
                        log.appendDouble(logHandle, (double) value, 0);
                        break;
                    case kString:
                        log.appendString(logHandle, (String) value, 0);
                        break;
                    case kRaw:
                        Byte[] valueAsBytes = (Byte[]) value;

                        byte[] primitiveByteArray = new byte[valueAsBytes.length];

                        for (int i = 0; i < valueAsBytes.length; i++)
                            primitiveByteArray[i] = valueAsBytes[i];

                        log.appendRaw(logHandle, primitiveByteArray, 0);

                        break;
                    case kBooleanArray:
                        Boolean[] valueAsBools = (Boolean[]) value;

                        boolean[] primitiveBoolArray = new boolean[valueAsBools.length];

                        for (int i = 0; i < valueAsBools.length; i++)
                            primitiveBoolArray[i] = valueAsBools[i];

                        log.appendBooleanArray(logHandle, primitiveBoolArray, 0);

                        break;
                    case kDoubleArray:
                        Double[] valueAsDoubles = (Double[]) value;

                        double[] primitiveDoubleArray = new double[valueAsDoubles.length];

                        for (int i = 0; i < valueAsDoubles.length; i++)
                            primitiveDoubleArray[i] = valueAsDoubles[i];

                        log.appendDoubleArray(logHandle, primitiveDoubleArray, 0);

                        break;
                    case kStringArray:
                        log.appendStringArray(logHandle, (String[]) value, 0);
                        break;
                        // won't have non-double numerical types
                    default:
                        log.appendString(logHandle, value.toString(), 0);
                        break;
                }
            }
        }
        prevVal = value;
    }

    /**
     * Updates data sinks with the given value.
     *
     * @param value Instance of type T to store internally and send to data sinks.
     */
    public void update(T value) {
        lambda = () -> value;

        update();
    }

    /**
     * Updates data sinks with the given value.
     *
     * @param lambda Lambda of type T to store internally and send to data sinks.
     */
    public void update(Supplier<T> lambda) {
        this.lambda = lambda;

        update();
    }
}
