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
package org.chsrobotics.lib.models;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import org.chsrobotics.lib.math.UtilityMath;
import org.ejml.simple.SimpleMatrix;

public class CoaxialSwerveModel<N extends Num> {
    // offsets from ROBOT to MODULE
    public static record CoaxialSwerveModuleOffsets<J extends Num>(
            Vector<J> xOffsets, Vector<J> yOffsets) {}

    // module angles in robot frame
    public static record CoaxialSwerveState<J extends Num>(
            double x,
            double y,
            double vX,
            double vY,
            double angle,
            double angularVelocity,
            Vector<J> steerAngle,
            Vector<J> steerAngularVelocity,
            Vector<J> driveAngularAccumulation,
            Vector<J> driveAngularVelocity) {}

    public static record CoaxialSwerveInput<J extends Num>(
            Vector<J> driveInput, Vector<J> steerInput) {
        private CoaxialSwerveInput<J> clamp(double driveMaxAbs, double steerMaxAbs) {
            var clDriveInput = driveInput;
            var clSteerInput = steerInput;

            for (int i = 0; i < clDriveInput.getNumCols(); i++) {
                clDriveInput.set(0, i, UtilityMath.clamp(driveMaxAbs, clDriveInput.get(0, i)));

                clSteerInput.set(0, i, UtilityMath.clamp(steerMaxAbs, clSteerInput.get(0, i)));
            }

            return new CoaxialSwerveInput<>(clDriveInput, clSteerInput);
        }
    }

    private final double robotMass;
    private final double robotMoment;

    private final double wheelDriveMoment;
    private final double wheelSteerMoment;

    private final DCMotor driveMotor;
    private final DCMotor steerMotor;

    private final double driveWheelRadius;

    private final CoaxialSwerveModuleOffsets<N> modulePositions;

    private final int numModules;

    private final int numStateVars;

    private final int numInputVars;

    private class NumInputs extends Num {
        @Override
        public int getNum() {
            return numInputVars;
        }
    }

    private class NumStates extends Num {
        @Override
        public int getNum() {
            return numStateVars;
        }
    }

    public CoaxialSwerveModel(
            double robotMass,
            double robotMoment,
            double wheelDriveMoment,
            double wheelSteerMoment,
            DCMotor driveMotor,
            DCMotor steerMotor,
            double driveWheelRadius,
            CoaxialSwerveModuleOffsets<N> modulePositions) {
        this.robotMass = robotMass;
        this.robotMoment = robotMoment;

        this.wheelDriveMoment = wheelDriveMoment;
        this.wheelSteerMoment = wheelSteerMoment;

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        this.driveWheelRadius = driveWheelRadius;

        this.modulePositions = modulePositions;

        // circumvents type erasure
        numModules = modulePositions.xOffsets.getNumRows();

        numInputVars = numModules * 2;
        // drive input, steer input

        numStateVars = 6 + (4 * numModules);
        // global states: x, y, vx, vy, angle, angular velocity

        // per-module states: steer angle, steer angular velocity,
        // drive angular accumulation, drive angular velocity
    }

    public CoaxialSwerveState<N> simulate(
            CoaxialSwerveState<N> state, CoaxialSwerveInput<N> input, double dtSeconds) {

        var clInput = input.clamp(driveMotor.nominalVoltageVolts, steerMotor.nominalVoltageVolts);

        SimpleMatrix stateAsVector = new SimpleMatrix(1, numStateVars);

        // set initial 6 values of state vector to global state variables
        stateAsVector.set(0, 0, state.x);
        stateAsVector.set(0, 1, state.y);
        stateAsVector.set(0, 2, state.vX);
        stateAsVector.set(0, 3, state.vY);
        stateAsVector.set(0, 4, state.angle);
        stateAsVector.set(0, 5, state.angularVelocity);

        // add a variable-size data footer to the vector, so that we can support arbitrary u_int
        // numbers of modules
        int index = 6;
        for (int i = 0; i < state.steerAngle.getNumCols(); i++) {
            stateAsVector.set(0, index, state.steerAngle.get(0, i));
            index++;

            stateAsVector.set(0, index, state.steerAngularVelocity.get(0, i));
            index++;

            stateAsVector.set(0, index, state.driveAngularAccumulation.get(0, i));

            stateAsVector.set(0, index, state.driveAngularVelocity.get(0, i));
            index++;
        }

        SimpleMatrix inputAsVector = new SimpleMatrix(1, numInputVars);

        // loop through each module again, changing all inputs to a single vector
        index = 0;
        for (int i = 0; i < clInput.driveInput.getNumCols(); i++) {
            inputAsVector.set(0, index, clInput.steerInput.get(0, i));
            index++;

            inputAsVector.set(0, index, clInput.driveInput.get(0, i));
            index++;
        }

        // RK4 numerical integration to get a solution to the IVP of our system
        Matrix<NumStates, N1> nextState =
                NumericalIntegration.rkdp(
                        this::getStateDerivative,
                        new Matrix<NumStates, N1>(stateAsVector),
                        new Matrix<NumInputs, N1>(inputAsVector),
                        dtSeconds);
        // being explicit on matrix dimensions out of an abundance of caution

        SimpleMatrix moduleSteerAngles = new SimpleMatrix(1, numModules);
        SimpleMatrix moduleSteerAngularVelocities = new SimpleMatrix(1, numModules);
        SimpleMatrix moduleDriveAngularAccumulation = new SimpleMatrix(1, numModules);
        SimpleMatrix moduleDriveAngularVelocities = new SimpleMatrix(1, numModules);

        // convert variable-size module data footer (everything after index 5) to their own vectors
        index = 6;
        for (int i = 0; i < numModules; i++) {
            moduleSteerAngles.set(0, i, nextState.get(0, index));
            index++;

            moduleSteerAngularVelocities.set(0, i, nextState.get(0, index));
            index++;

            moduleDriveAngularAccumulation.set(0, i, nextState.get(0, index));
            index++;

            moduleDriveAngularVelocities.set(0, i, nextState.get(0, index));
            index++;
        }

        return new CoaxialSwerveState<>(
                nextState.get(0, 0), // x
                nextState.get(0, 1), // y
                nextState.get(0, 2), // vx
                nextState.get(0, 3), // vy
                nextState.get(0, 4), // angle
                nextState.get(0, 5), // angular velocity
                new Vector<>(moduleSteerAngles),
                new Vector<>(moduleSteerAngularVelocities),
                new Vector<>(moduleDriveAngularAccumulation),
                new Vector<>(moduleDriveAngularVelocities));
    }

    // returns the derivative of the state, given current state and an input
    private Matrix<NumStates, N1> getStateDerivative(
            Matrix<NumStates, N1> x, Matrix<NumInputs, N1> u) {
        SimpleMatrix derivState = new SimpleMatrix(1, numStateVars);

        derivState.set(0, 0, x.get(0, 2)); // move down vx
        derivState.set(0, 1, x.get(0, 3)); // move down vy

        derivState.set(0, 4, x.get(0, 5)); // move down angular vel

        Translation2d forceSum = new Translation2d();

        double torqueSum = 0;

        // calculations on still-packed variable data footer
        int stateVarIndex = 6;
        int inputVarIndex = 0;
        for (int i = 0; i < numModules; i++) {
            // input var = steer
            double steerU = u.get(0, inputVarIndex);
            inputVarIndex++;

            // input var = drive
            double driveU = u.get(0, inputVarIndex);
            inputVarIndex++;

            // state var = steer angle
            double steerAngle = x.get(0, stateVarIndex);

            derivState.set(
                    0, stateVarIndex, x.get(0, stateVarIndex + 1)); // move down angular velocity
            stateVarIndex++;

            // state var = steer angular velocity
            double steerTorque =
                    steerMotor.getTorque(steerMotor.getCurrent(x.get(0, stateVarIndex), steerU));

            derivState.set(0, i, steerTorque / wheelSteerMoment); // torque / moment = angular accel
            stateVarIndex++;

            // state var = drive angle
            derivState.set(0, i, x.get(0, stateVarIndex + 1)); // move down angular velocity
            stateVarIndex++;

            // state var = drive angular velocity
            double driveTorque =
                    driveMotor.getTorque(driveMotor.getCurrent(x.get(0, stateVarIndex), driveU));

            double forceAlongWheelVec = driveTorque / driveWheelRadius;

            // module angles are in robot frame already, no need to transform our forces
            forceSum =
                    forceSum.plus(
                            new Translation2d(forceAlongWheelVec, new Rotation2d(steerAngle)));

            // torque = force vector X (cross product) radius vector

            // A X B = |A||B|sin(theta) where theta is angle between vectors
            // this definition isn't TECHNICALLY correct but good enough for us

            Translation2d radiusVector =
                    new Translation2d(
                            -modulePositions.xOffsets.get(0, i),
                            -modulePositions.yOffsets.get(0, i));

            double angleBetween =
                    new Rotation2d(steerAngle).minus(radiusVector.getAngle()).getRadians();

            torqueSum +=
                    radiusVector.getNorm() * Math.abs(forceAlongWheelVec) * Math.sin(angleBetween);

            derivState.set(
                    0,
                    stateVarIndex,
                    driveTorque / wheelDriveMoment); // torque / moment = angular accel
            stateVarIndex++;
        }

        // rotate forces to world-relative
        forceSum = forceSum.rotateBy(new Rotation2d(-x.get(0, 0)));

        // set global state derivs to sum of forces/ torques from modules
        derivState.set(0, 2, forceSum.getX() / robotMass); // force / mass = accel
        derivState.set(0, 3, forceSum.getY() / robotMass);

        derivState.set(0, 5, torqueSum / robotMoment); // torque / moment = angular accel

        return new Matrix<>(derivState);
    }
}
