/*
    https://github.com/kousheekc/Kinematics

    MIT License

    Copyright (c) 2022 Kousheek Chakraborty

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

#pragma once

#ifndef KINEMATICS_h
#define KINEMATICS_h

#include "MatrixUtils.h"

class Kinematics {
private:
    int num_of_joints;
    int num_of_joints_declared;
    double joint_screw_axes[6][6];
    double initial_end_effector_pose[4][4];
    MatrixUtils mat_utils;

public:
    Kinematics(int num_of_joints_);

    void add_joint_axis(double s1, double s2, double s3, double s4, double s5, double s6);
    void add_initial_end_effector_pose(double m11, double m12, double m13, double m14, double m21, double m22, double m23, double m24, double m31, double m32, double m33, double m34, double m41, double m42, double m43, double m44);

    void forward(double* joint_angles, double* transform);
    void inverse(double* transform, double* jac, double* pinv, double* A_t, double* AA_t, double* A_tA, double* initial_joint_angles, double ew, double ev, double max_iterations, double* joint_angles);
    void jacobian(double* joint_angles, double* jacobian);
};

#endif