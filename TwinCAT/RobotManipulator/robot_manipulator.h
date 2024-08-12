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

#ifndef ROBOT_MANIPULATOR_H
#define ROBOT_MANIPULATOR_H

#include "ArduinoEigen/ArduinoEigen.h"
//#include <ArduinoEigen.h>  // Include ArduinoEigen library for matrix operations

// Struct to define joint parameters
struct JointParameters {
    double a;      // Link length
    double alpha;  // Twist angle in radians
    double d;      // Offset along the z-axis
    double theta;  // Joint angle in radians
};

// Struct to store forward kinematics result
struct ForwardKinematicsResult {
    Eigen::Vector3d position;     // End-effector position
    Eigen::Matrix3d orientation;  // End-effector orientation
};

// Struct to store inverse kinematics result
struct InverseKinematicsResult {
    double jointAngles[6];  // Joint angles (assuming a maximum of 6 joints)
    bool success;           // Indicates if the inverse kinematics computation was successful
};

class RobotManipulator {
public:
    // Constructor
    RobotManipulator(const JointParameters joints[], int numJoints);

    // Forward kinematics
    ForwardKinematicsResult forwardKinematics(const double jointAngles[]);

    // Inverse kinematics using Jacobian Transpose method
    InverseKinematicsResult inverseKinematics(const Eigen::Vector3d& targetPosition,
                                              const Eigen::Matrix3d& targetOrientation,
                                              double tolerance = 1e-5, int maxIterations = 100);

private:
    int numJoints_;               // Number of joints
    JointParameters* jointParams_; // Array to store joint parameters

    // Function to compute a transformation matrix for a given set of DH parameters
    void computeTransform(Eigen::Matrix4d& transform, double a, double alpha, double d, double theta);

    // Function to compute the Jacobian matrix
    void computeJacobian(const Eigen::Vector3d& endEffectorPosition,
                         const Eigen::Matrix3d& endEffectorOrientation,
                         Eigen::MatrixXd& jacobian);

    // Function to check if the difference between two vectors is below a certain tolerance
    bool isConverged(const Eigen::VectorXd& error, double tolerance);
};

#endif // ROBOT_MANIPULATOR_H