﻿// Kinematics.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// https://github.com/kousheekc/Kinematics
#include <iostream>

#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 6

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))

struct Pose {
    double position[3];
    double orientation[3]; // Roll, Pitch, Yaw
};

Pose extractPose(const double transform[4][4]) {
    Pose pose;

    // Extract position
    pose.position[0] = transform[0][3];
    pose.position[1] = transform[1][3];
    pose.position[2] = transform[2][3];

    // Extract orientation (roll, pitch, yaw)
    double sy = sqrt(transform[0][0] * transform[0][0] + transform[1][0] * transform[1][0]);

    bool singular = sy < 1e-6; // If sy is close to zero, singularity is detected

    if (!singular) {
        pose.orientation[0] = atan2(transform[2][1], transform[2][2]); // Roll
        pose.orientation[1] = atan2(-transform[2][0], sy);              // Pitch
        pose.orientation[2] = atan2(transform[1][0], transform[0][0]);  // Yaw
    }
    else {
        pose.orientation[0] = atan2(-transform[1][2], transform[1][1]); // Roll
        pose.orientation[1] = atan2(-transform[2][0], sy);              // Pitch
        pose.orientation[2] = 0;                                        // Yaw
    }

    return pose;
}

void createTransformMatrix(const Pose& pose, double transform[4][4]) {
    double roll = pose.orientation[0];
    double pitch = pose.orientation[1];
    double yaw = pose.orientation[2];

    // Calculate rotation matrix components
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    transform[0][0] = cy * cp;
    transform[0][1] = cy * sp * sr - sy * cr;
    transform[0][2] = cy * sp * cr + sy * sr;
    transform[0][3] = pose.position[0];

    transform[1][0] = sy * cp;
    transform[1][1] = sy * sp * sr + cy * cr;
    transform[1][2] = sy * sp * cr - cy * sr;
    transform[1][3] = pose.position[1];

    transform[2][0] = -sp;
    transform[2][1] = cp * sr;
    transform[2][2] = cp * cr;
    transform[2][3] = pose.position[2];

    transform[3][0] = 0;
    transform[3][1] = 0;
    transform[3][2] = 0;
    transform[3][3] = 1;
}

int main()
{
    //for (int _j=-180; _j <= 180; _j += 45) {
    Kinematics kin(N);
    MatrixUtils mat_utils;

    double L1 = 64.2;
    double L2 = 169.77;
    double L3 = 305;
    double L4 = 222.63;
    double L5 = 36.25;

    //// screw axis expressed in "s-frame" 
    kin.add_joint_axis(0, 0, 1, 0, 0, 0); // axis 1
    kin.add_joint_axis(0, 1, 0, -L2, 0, L1); // axis 2
    kin.add_joint_axis(0, 1, 0, -L2 - L3, 0, L1); // axis 3
    kin.add_joint_axis(0, 0, 1, 0, -L1, 0); // axis 4
    kin.add_joint_axis(0, 1, 0, -L2 - L3 - L4, 0, L1); // axis 5
    kin.add_joint_axis(0, 0, 1, 0, -L1, 0); // axis 6

    kin.add_initial_end_effector_pose(1, 0, 0, L1,
        0, 1, 0, 0,
        0, 0, 1, L2 + L3 + L4 + L5,
        0, 0, 0, 1);

    // expected results
    // 0,0,0,0,0,0 | x64.2, y0, z733.65
    // 90,0,0,0,0,0 | x0, y64.2, z733.65
    // 0,90,0,0,0,0 | x628.08, y0, z169.77
    // 0,0,90,0,0,0 | x323.08, y0, z474.77
    // 0,0,0,90,0,0 | x64.2, y0, z733.65, a0, b0, c90
    // 0,0,0,0,90,0 | x100.45, 0, z697.4
    // 0,0,0,0,0,90 | x64.2, y0, z733.65, a0, b0, c90
    double joint_angles_in[N] = { DEG_TO_RAD(0.0), DEG_TO_RAD(0.0), DEG_TO_RAD(90.0),
                                    DEG_TO_RAD(0.0), DEG_TO_RAD(45.0), DEG_TO_RAD(0.0) };
    double transform[4][4];

    kin.forward(joint_angles_in, (double*)transform);
    //mat_utils.print_matrix((double*)transform, 4, 4/*, "Transform"*/);
    //std::cout << std::endl;

    Pose pose = extractPose(transform);

    std::cout << "Position: (" << pose.position[0] << ", " << pose.position[1] << ", " << pose.position[2] << ")\n";
    std::cout << "Orientation: (Roll: " << RAD_TO_DEG(pose.orientation[0])
        << ", Pitch: " << RAD_TO_DEG(pose.orientation[1])
        << ", Yaw: " << RAD_TO_DEG(pose.orientation[2]) << ")\n";

    //////
    ////////////////////////
    //////

    createTransformMatrix(pose, transform);

    //std::cout << "Transformation Matrix:\n";
    //for (int i = 0; i < 4; ++i) {
    //    for (int j = 0; j < 4; ++j) {
    //        std::cout << transform[i][j] << " ";
    //    }
    //    std::cout << "\n";
    //}

    double jac[6][N];
    double jac_t[6][N];
    double AA_t[6][6];
    double A_tA[N][N];
    double pinv[N][6];

    double joint_angles_out[N];

    /*
    (double*) desired end effector pose
    (double*) placeholder for Jacobian
    (double*) placeholder for pseudo inverse
    (double*) placeholder for pseudo inverse transpose
    (double*) placeholder for pseudo inverse * pseudo inverse transpose
    (double*) placeholder for pseudo inverse transpose * pseudo inverse
    (double*) initial joint angles
    (double) acceptable error for rotation component
    (double) acceptable error for position component
    (double) maximum iterations for Newton Raphson method
    (double*) placeholder for joint angles output
    */
    double joint_angles_old[N];;
    for (int _l = 0; _l <= N; _l++) {
        joint_angles_old[_l] = joint_angles_in[_l];
    }

    kin.inverse((double*)transform, (double*)jac, (double*)pinv, (double*)jac_t,
        (double*)AA_t, (double*)A_tA, joint_angles_old, 0.01, 0.001, 20,
        joint_angles_out);
    mat_utils.print_matrix(joint_angles_in, 1, N/*, "Joint angles"*/);
    mat_utils.print_matrix(joint_angles_out, 1, N/*, "Joint angles"*/);
    //}
}