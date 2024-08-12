// Kinematics.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// https://github.com/kousheekc/Kinematics
#include <iostream>

#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 6

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))

int main()
{
    Kinematics kin(N);
    MatrixUtils mat_utils;

    double L1 = 64.2;
    double L2 = 169.77;
    double L3 = 305;
    double L4 = 222.63;
    double L5 = 36.25;

    // calculated
    //kin.add_joint_axis(0, 0, 1,  0, 0, 0); // axis 1
    //kin.add_joint_axis(0, 1, 0, L1, 0, L2); // axis 2
    //kin.add_joint_axis(0, 1, 0, L1, 0, L2+L3); // axis 3
    //kin.add_joint_axis(0, 0, 1, L1, 0, L2+L3); // axis 4
    //kin.add_joint_axis(0, 1, 0, L1, 0, L2+L3+L4); // axis 5
    //kin.add_joint_axis(0, 0, 1, L1, 0, L2+L3+L4); // axis 6

    // trial and error but returned values are inverse
    //kin.add_joint_axis(0, 0, 1, 0, 0, 0); // axis 1
    //kin.add_joint_axis(0, 1, 0, 0, 0, -L5-L4-L3); // axis 2
    //kin.add_joint_axis(0, 1, 0, 0, 0, -L5-L4); // axis 3
    //kin.add_joint_axis(0, 0, 1, -L1, 0, 0); // axis 4
    //kin.add_joint_axis(0, 1, 0, 0, 0, -L5); // axis 5
    //kin.add_joint_axis(0, 0, 1, -L1, 0, 0); // axis 6

    kin.add_joint_axis(0, 0, 1, 0, 0, 0); // axis 1
    kin.add_joint_axis(0, 1, 0, 0, 0, -L5 - L4 - L3); // axis 2
    kin.add_joint_axis(0, 1, 0, 0, 0, -L5 - L4); // axis 3
    kin.add_joint_axis(0, 0, 1, -L1, 0, 0); // axis 4
    kin.add_joint_axis(0, 1, 0, 0, 0, -L5); // axis 5
    kin.add_joint_axis(0, 0, 1, -L1, 0, 0); // axis 6

    kin.add_initial_end_effector_pose(  1,  0,  0,  L1,
                                        0,  1,  0,  0,
                                        0,  0,  1,  L2+L3+L4+L5,
                                        0,  0,  0,  1);

    double joint_angles_in[N] = {   DEG_TO_RAD(0.0), DEG_TO_RAD(90.0), DEG_TO_RAD(0.0), 
                                    DEG_TO_RAD(0.0), DEG_TO_RAD(0.0), DEG_TO_RAD(0.0) };
    double transform[4][4];

    kin.forward(joint_angles_in, (double*)transform);
    mat_utils.print_matrix((double*)transform, 4, 4/*, "Transform"*/);


    //////////////////////////////////////////////////

    //double desired_transform[4][4] = {
    //                                    {0, 1,  0,     -5},
    //                                    {1, 0,  0,      4},
    //                                    {0, 0, -1, 1.6858},
    //                                    {0, 0,  0,      1}
    //                                    };

    double jac[6][N];
    double jac_t[6][N];
    double AA_t[6][6];
    double A_tA[N][N];
    double pinv[N][6];

    //double joint_angles_0[N] = { 1.0, 2.5, 3 };
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
    kin.inverse((double*)transform, (double*)jac, (double*)pinv, (double*)jac_t, 
                (double*)AA_t, (double*)A_tA, joint_angles_in, 0.01, 0.001, 20, 
                joint_angles_out);
    //mat_utils.print_matrix(joint_angles_in, 1, N/*, "Joint angles"*/);
    //mat_utils.print_matrix(joint_angles_out, 1, N/*, "Joint angles"*/);
}

/*
forward
#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 3

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    Kinematics kin(N);
    MatrixUtils mat_utils;

    kin.add_joint_axis(0, 0,  1,  4, 0,    0);
    kin.add_joint_axis(0, 0,  0,  0, 1,    0);
    kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

    kin.add_initial_end_effector_pose(-1, 0,  0, 0,
                                       0, 1,  0, 6,
                                       0, 0, -1, 2,
                                       0, 0,  0, 1);

    float joint_angles[N] = {PI/2.0, 3, PI};
    float transform[4][4];

    kin.forward(joint_angles, (float*)transform);
    mat_utils.print_matrix((float*)transform, 4, 4, "Transform");

    // Output
    // Transform
    // 0.00    1.00    0.00    -5.00
    // 1.00    -0.00   0.00    4.00
    // 0.00    0.00    -1.00   1.69
    // 0.00    0.00    0.00    1.00
}

void loop() {
    // put your main code here, to run repeatedly:

}

*/

/*
reverse
#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 3

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    Kinematics kin(N);
    MatrixUtils mat_utils;

    kin.add_joint_axis(0, 0,  1,  4, 0,    0);
    kin.add_joint_axis(0, 0,  0,  0, 1,    0);
    kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

    kin.add_initial_end_effector_pose(-1, 0,  0, 0,
                                       0, 1,  0, 6,
                                       0, 0, -1, 2,
                                       0, 0,  0, 1);

    float desired_transform[4][4] = {
        {0, 1,  0,     -5},
        {1, 0,  0,      4},
        {0, 0, -1, 1.6858},
        {0, 0,  0,      1}
    };

    float jac[6][N];
    float jac_t[6][N];
    float AA_t[6][6];
    float A_tA[N][N];
    float pinv[N][6];

    float joint_angles_0[N] = {1.0, 2.5, 3};
    float joint_angles[N];

    kin.inverse((float*)desired_transform, (float*)jac, (float*)pinv, (float*)jac_t, (float*)AA_t, (float*)A_tA, joint_angles_0, 0.01, 0.001, 20, joint_angles);
    mat_utils.print_matrix(joint_angles, 1, N, "Joint angles");
}

void loop() {
  // put your main code here, to run repeatedly:

}

*/