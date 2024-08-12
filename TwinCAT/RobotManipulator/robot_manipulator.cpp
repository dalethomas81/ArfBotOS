/*
MIT License

Copyright (c) 2023 Adam Vadala-Roth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "robot_manipulator.h"

// Constructor implementation
RobotManipulator::RobotManipulator(const JointParameters joints[], int numJoints)
    : numJoints_(numJoints) {
    // Allocate memory for joint parameters
    jointParams_ = new JointParameters[numJoints_];

    // Copy joint parameters
    for (int i = 0; i < numJoints_; ++i) {
        jointParams_[i] = joints[i];
    }
}

// Forward kinematics implementation
ForwardKinematicsResult RobotManipulator::forwardKinematics(const double jointAngles[]) {
    ForwardKinematicsResult result;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    // Iterate through each joint
    for (int i = 0; i < numJoints_; ++i) {
        Eigen::Matrix4d tempTransform;
        // Compute the transformation matrix for the current joint
        computeTransform(tempTransform, jointParams_[i].a, jointParams_[i].alpha, jointParams_[i].d, jointAngles[i]);
        // Multiply the transformation matrix with the cumulative transformation
        transform = transform * tempTransform;
    }

    // Extract position and orientation from the resulting matrix
    result.position = transform.block<3, 1>(0, 3);
    result.orientation = transform.block<3, 3>(0, 0);

    return result;
}

// Inverse kinematics using Jacobian Transpose method
InverseKinematicsResult RobotManipulator::inverseKinematics(const Eigen::Vector3d& targetPosition,
                                                            const Eigen::Matrix3d& targetOrientation,
                                                            double tolerance, int maxIterations) {
    InverseKinematicsResult result;
    Eigen::Vector3d endEffectorPosition;
    Eigen::Matrix3d endEffectorOrientation;
    Eigen::MatrixXd jacobian(6, numJoints_);
    Eigen::VectorXd error(6);

    // Initialize result
    result.success = false;

    // Iterate through the maximum number of iterations
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // Perform forward kinematics to compute current end-effector pose
        ForwardKinematicsResult fkResult = forwardKinematics(result.jointAngles);

        // Compute error in position and orientation
        endEffectorPosition = fkResult.position;
        endEffectorOrientation = fkResult.orientation;

        error.block<3, 1>(0, 0) = targetPosition - endEffectorPosition;
        error.block<3, 1>(3, 0) = 0.5 * (targetOrientation.col(0).cross(endEffectorOrientation.col(0)) +
                                         targetOrientation.col(1).cross(endEffectorOrientation.col(1)) +
                                         targetOrientation.col(2).cross(endEffectorOrientation.col(2)));

        // Check for convergence
        if (isConverged(error, tolerance)) {
            result.success = true;
            return result;
        }

        // Compute Jacobian matrix
        computeJacobian(endEffectorPosition, endEffectorOrientation, jacobian);

        // Update joint angles using Jacobian Transpose method
        for (int i = 0; i < numJoints_; ++i) {
            result.jointAngles[i] += jacobian.transpose().row(i) * error;
        }
    }

    return result;
}


// Function to compute a transformation matrix for a given set of DH parameters
void RobotManipulator::computeTransform(Eigen::Matrix4d& transform, double a, double alpha, double d, double theta) {
    // Populate the transformation matrix using DH parameters
    transform << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
                 sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                 0, sin(alpha), cos(alpha), d,
                 0, 0, 0, 1;
}

// Function to compute the Jacobian matrix
void RobotManipulator::computeJacobian(const Eigen::Vector3d& endEffectorPosition,
                                       const Eigen::Matrix3d& endEffectorOrientation,
                                       Eigen::MatrixXd& jacobian) {
    Eigen::Vector3d jointPosition;
    Eigen::Matrix3d jointOrientation;

    Eigen::Vector3d zAxis;
    Eigen::Vector3d jointAxis;

    // Iterate through each joint to compute Jacobian columns
    for (int i = 0; i < numJoints_; ++i) {
        // Compute joint position and orientation
        jointPosition = Eigen::Vector3d(jointParams_[i].a, 0, 0);
        jointOrientation = Eigen::Matrix3d::Identity();

        // Compute joint axis in the global frame
        zAxis = jointOrientation.col(2);
        jointAxis = zAxis.cross(endEffectorPosition - jointPosition);

        // Populate the translational part of the Jacobian
        jacobian.block<3, 1>(0, i) = jointAxis;
        // Populate the rotational part of the Jacobian
        jacobian.block<3, 1>(3, i) = zAxis;
    }
}

// Function to check if the difference between two vectors is below a certain tolerance
bool RobotManipulator::isConverged(const Eigen::VectorXd& error, double tolerance) {
    // Compute the Euclidean norm of the error vector
    double norm = error.norm();
    // Check if the norm is below the specified tolerance
    return (norm < tolerance);
}
